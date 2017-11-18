#include "MissionGenerator.hpp"
#include <sstream>
#include <organization_model/OrganizationModel.hpp>
#include "../symbols/constants/Location.hpp"
#include <organization_model/vocabularies/VRP.hpp>

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


    for(const Coord2D& coord : vrp.getNodeCoordinates())
    {
        // Add locations -- nodes
        base::Point position;
        position.x() = coord.x;
        position.y() = coord.y;
        position.z() = 0;
        bool metricDefinition = true;

        std::stringstream ss;
        ss << "l" << ++idx;

        constants::Location::Ptr location = constants::Location::create(ss.str(), position);
        mission->addConstant(location);

        std::stringstream ssFrom;
        ssFrom << "t" << idx << "-0";
        std::stringstream ssTo;
        ssTo << "t" << idx << "-1";
        TimePoint::Ptr from = tcn->getOrCreateTimePoint(ssFrom.str());
        TimePoint::Ptr to = tcn->getOrCreateTimePoint(ssTo.str());

        if(!endOfFirstInterval)
        {
            endOfFirstInterval = to;
        } else {
            try {
                // making sure that depot is the first to be considered
                solvers::Constraint::Ptr constraint0 = tcn->addQualitativeConstraint(endOfFirstInterval, from, QualitativeTimePointConstraint::Less);
                mission->addConstraint(constraint0);
            } catch(...)
            {}
        }


        TemporalAssertion::Ptr temporalAssertion = mission->addResourceLocationCardinalityConstraint(location,
                from, to,
                vocabulary::VRP::Commodity(),
                vrp.getDemands()[idx-1],
                owlapi::model::OWLCardinalityRestriction::MIN);
    }

    return mission;
}

} // end namespace benchmark
} // end namespace templ
