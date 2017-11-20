#include "MissionGenerator.hpp"
#include <sstream>
#include <organization_model/OrganizationModel.hpp>
#include <organization_model/vocabularies/VRP.hpp>

#include "../symbols/constants/Location.hpp"

using namespace organization_model;
using namespace templ::symbols;
using namespace templ::solvers::temporal::point_algebra;
using namespace templ::solvers::temporal;

namespace templ {
namespace benchmark {

owlapi::model::IRI MissionGenerator::mVRPOntology = "http://www.rock-robotics.org/2017/11/vrp#";

Mission::Ptr MissionGenerator::convert(const VRPProblem& vrp)
{
    OrganizationModel::Ptr om(new OrganizationModel(mVRPOntology));
    Mission::Ptr mission(new Mission(om, vrp.getName()));

    solvers::temporal::TemporalConstraintNetwork::Ptr tcn = mission->getTemporalConstraintNetwork();
    // idx
    // Add constants
    size_t idx = 0;
    TimePoint::Ptr endOfFirstInterval;
    organization_model::ModelPool availableResources;
    availableResources[vocabulary::VRP::Commodity()] = vrp.getTotalDemand();
    availableResources[vocabulary::VRP::Vehicle()] = vrp.getVehicles();
    mission->setAvailableResources(availableResources);

    std::map<Coord2D, constants::Location::Ptr> coordLocationMap;
    std::vector<TimePoint::Ptr> timepoints;
    for(const Coord2D& coord : vrp.getNodeCoordinates())
    {
        // Add locations -- nodes
        base::Point position;
        position.x() = coord.x;
        position.y() = coord.y;
        position.z() = 0;

        std::stringstream ss;
        ss << "l" << ++idx;

        constants::Location::Ptr location = constants::Location::create(ss.str(), position);
        mission->addConstant(location);

        coordLocationMap[coord] = location;

        std::stringstream ssFrom;
        ssFrom << "t" << idx << "-0";
        std::stringstream ssTo;
        ssTo << "t" << idx << "-1";
        TimePoint::Ptr from = tcn->getOrCreateTimePoint(ssFrom.str());
        timepoints.push_back(from);
        TimePoint::Ptr to = tcn->getOrCreateTimePoint(ssTo.str());
        timepoints.push_back(to);

        mission->addResourceLocationCardinalityConstraint(location,
                from, to,
                vocabulary::VRP::Commodity(),
                vrp.getDemands()[idx-1],
                owlapi::model::OWLCardinalityRestriction::MIN);
    }

    size_t depotIdx = 0;
    TimePoint::Ptr startFrom;
    TimePoint::Ptr startTo;
    for(const Coord2D& coord : vrp.getDepots())
    {
        std::map<Coord2D, constants::Location::Ptr>::const_iterator cit = coordLocationMap.find(coord);
        constants::Location::Ptr depotLocation;
        if(cit != coordLocationMap.end())
        {
            depotLocation = cit->second;
            startFrom = timepoints[0];
            startTo = timepoints[1];
        } else {

            base::Point position;
            position.x() = coord.x;
            position.y() = coord.y;
            position.z() = 0;

            startFrom = tcn->getOrCreateTimePoint("t-init-0");
            startTo = tcn->getOrCreateTimePoint("t-init-1");

            std::stringstream ss;
            ss << "depot" << depotIdx++;

            depotLocation = constants::Location::create(ss.str(), position);
            mission->addConstant(depotLocation);
        }

        // Making sure vehicles are available at starting locations
        mission->addResourceLocationCardinalityConstraint(depotLocation,
                startFrom,
                startTo,
                vocabulary::VRP::Vehicle(),
                vrp.getVehicles(),
                owlapi::model::OWLCardinalityRestriction::MAX);

        mission->addResourceLocationCardinalityConstraint(depotLocation,
                startFrom,
                startTo,
                vocabulary::VRP::Vehicle(),
                vrp.getVehicles(),
                owlapi::model::OWLCardinalityRestriction::MIN);

        // Making sure commodities are available at starting locations
        mission->addResourceLocationCardinalityConstraint(depotLocation,
                startFrom,
                startTo,
                vocabulary::VRP::Commodity(),
                vrp.getTotalDemand(),
                owlapi::model::OWLCardinalityRestriction::MIN);

        mission->addResourceLocationCardinalityConstraint(depotLocation,
                startFrom,
                startTo,
                vocabulary::VRP::Commodity(),
                vrp.getTotalDemand(),
                owlapi::model::OWLCardinalityRestriction::MAX);
    }

    for(point_algebra::TimePoint::Ptr t : timepoints)
    {
        try {
            // making sure that depot is the first to be considered
            solvers::Constraint::Ptr constraint0 = tcn->addQualitativeConstraint(startTo, t, QualitativeTimePointConstraint::Less);
            mission->addConstraint(constraint0);
        } catch(...)
        {}
    }

    return mission;
}

} // end namespace benchmark
} // end namespace templ
