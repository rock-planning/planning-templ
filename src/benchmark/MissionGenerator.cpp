#include "MissionGenerator.hpp"
#include <sstream>
#include <organization_model/OrganizationModel.hpp>
#include <organization_model/vocabularies/VRP.hpp>
#include <graph_analysis/algorithms/MultiCommodityMinCostFlow.hpp>

#include "../symbols/constants/Location.hpp"
#include <qxcfg/Configuration.hpp>
#include "../SpaceTime.hpp"
#include "../solvers/transshipment/MinCostFlow.hpp"

using namespace organization_model;
using namespace templ::symbols;
using namespace templ::solvers::temporal::point_algebra;
using namespace templ::solvers::temporal;

namespace templ {
namespace benchmark {

owlapi::model::IRI MissionGenerator::mVRPOntology = "http://www.rock-robotics.org/2017/11/vrp#";

MissionGenerator::MissionGenerator(const std::string& configurationFile)
{
    if(!configurationFile.empty())
    {
        loadConfiguration(configurationFile);
    }
}

void MissionGenerator::loadConfiguration(const std::string& configurationFile)
{
    qxcfg::Configuration configuration(configurationFile);

    mAreaX = configuration.getValueAs<double>("MissionGenerator/area/x");
    mAreaY = configuration.getValueAs<double>("MissionGenerator/area/y");
    mAreaZ = configuration.getValueAs<double>("MissionGenerator/area/z");

    mNumberOfLocations = configuration.getValueAs<size_t>("MissionGenerator/locations",0);
    mNumberOfTimepoints = configuration.getValueAs<size_t>("MissionGenerator/timepoints",0);
    owlapi::model::IRI organizationModelIRI = configuration.getValueAs<std::string>("MissionGenerator/organization_model");

    mpOrganizationModel = organization_model::OrganizationModel::getInstance(organizationModelIRI);

    organization_model::OrganizationModelAsk ask(mpOrganizationModel);
    owlapi::model::IRIList agents = ask.ontology().allSubClassesOf(organization_model::vocabulary::OM::Actor());

    mMinPool.clear();
    mMaxPool.clear();
    // for each agent type -- fragment
    for(owlapi::model::IRI agent : agents)
    {
        {
            std::stringstream ss;
            ss << "MissionGenerator/agentPool/min/" << agent.getFragment();
            mMinPool[agent] = configuration.getValueAs<size_t>(ss.str(),0);
        }
        {
            std::stringstream ss;
            ss << "MissionGenerator/agentPool/max/" << agent.getFragment();
            mMaxPool[agent] = configuration.getValueAs<size_t>(ss.str(),0);
        }
    }

    mMinAgentCount = configuration.getValueAs<size_t>("MissionGenerator/minAgents",0);
    mMaxAgentCount = configuration.getValueAs<size_t>("MissionGenerator/maxAgents",0);

    mRatioImmobileMobile = configuration.getValueAs<double>("MissionGenerato/ratioImmobileMobile",0.5);
    mRatioFunctionaltiesAgents = configuration.getValueAs<double>("MissionGenerator/ratioFunctionalitiesAgents",0.8);

    mSamplingDensity = configuration.getValueAs<double>("MissionGenerator/samplingDensity",0.2);
    mUseFunctionalities = configuration.getValueAs<double>("MissionGenerator/useFunctionalties",true);
}

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

    DataPropertyAssignment da(vocabulary::VRP::Vehicle(), vocabulary::OM::transportCapacity(), vrp.getCapacity());
    mission->addDataPropertyAssignment(da);

    for(point_algebra::TimePoint::Ptr t : timepoints)
    {
        try {
            // making sure that depot is the first to be considered
            Constraint::Ptr constraint0 = tcn->addQualitativeConstraint(startTo, t, QualitativeTimePointConstraint::Less);
            mission->addConstraint(constraint0);
        } catch(...)
        {}
    }

    return mission;
}

Mission::Ptr MissionGenerator::generate()
{
    SpaceTime::Network network = generateNetwork();
    Mission::Ptr mission = sampleFromNetwork(network);
    // create a network
    //organization_model::ModelPool
    return mission;
}

Mission::Ptr MissionGenerator::sampleFromNetwork(const SpaceTime::Network& network)
{
    Mission::Ptr mission = make_shared<Mission>(mpOrganizationModel,
            "autogenerated-mission");
    mission->setAvailableResources(mAgentPool);

    using namespace graph_analysis;
    using namespace symbols::constants;
    symbols::constants::Location::PtrList locations = network.getValues();
    solvers::temporal::point_algebra::TimePoint::PtrList timepoints = network.getTimepoints();

    for(Location::Ptr location : locations)
    {
        mission->addConstant(location);
    }

    // Prepare initial locations
    for(size_t l = 0; l < locations.size(); ++l)
    {
        Location::Ptr location = locations[l];
        // A tuple with information about the allocation -> transform into a
        // Persistence Condition
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr tuple =
            network.tupleByKeys(location, timepoints[0]);

        organization_model::ModelPool agentPool = tuple->getModelPool({ RoleInfo::ASSIGNED, RoleInfo::AVAILABLE});
        for(const std::pair<owlapi::model::IRI, size_t>& v : agentPool)
        {
            mission->addResourceLocationCardinalityConstraint(location,
                    timepoints[0],
                    timepoints[1],
                    v.first,
                    v.second,
                    owlapi::model::OWLCardinalityRestriction::MIN);
        }
    }

    // sampling density
    Edge::PtrList relevantEdges;
    EdgeIterator::Ptr edgeIt = network.getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {
        Edge::Ptr edge = edgeIt->current();
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr fromTuple =
            dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(edge->getSourceVertex());
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr toTuple =
            dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(edge->getSourceVertex());

        if(fromTuple->second() == timepoints[0] || toTuple->second() ==
                timepoints[0])
        {
            continue;
        }

        if(*fromTuple->first() != *toTuple->first())
        {
            continue;
        }

        // only use intervals for the same location
        relevantEdges.push_back(edge);
    }

    // Sample from the rest
    size_t minCount = relevantEdges.size()*mSamplingDensity;
    while(minCount > 0)
    {
        size_t edgeIdx = rand() % minCount;
        const Edge::Ptr& edge = relevantEdges[edgeIdx];
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr fromTuple =
            dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(edge->getSourceVertex());
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr toTuple =
            dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(edge->getTargetVertex());

        organization_model::ModelPool agentPool = fromTuple->getModelPool({ RoleInfo::ASSIGNED, RoleInfo::AVAILABLE});
        try {
            for(const std::pair<owlapi::model::IRI, size_t>& v : agentPool)
            {
                std::cout << "min " << fromTuple->first()->toString() << " " <<
                    fromTuple->second()->toString() << std::endl;

                mission->addResourceLocationCardinalityConstraint(fromTuple->first(),
                        fromTuple->second(),
                        toTuple->second(),
                        v.first,
                        v.second,
                        owlapi::model::OWLCardinalityRestriction::MIN);
            }
        } catch(...)
        {
            // retry
            continue;
        }
        --minCount;
    }

    solvers::temporal::TemporalConstraintNetwork::Ptr tcn = mission->getTemporalConstraintNetwork();

    for(size_t t = 1; t < timepoints.size(); ++t)
    {
        try {
            // making sure that depot is the first to be considered
            Constraint::Ptr constraint0 =
                tcn->addQualitativeConstraint(timepoints[t-1], timepoints[t],
                        QualitativeTimePointConstraint::Less);
            mission->addConstraint(constraint0);
        } catch(...)
        {}
    }

    return mission;
}

SpaceTime::Network MissionGenerator::generateNetwork()
{
    using namespace graph_analysis;
    using namespace graph_analysis::algorithms;
    using namespace symbols::constants;

    Location::PtrList locations;
    for(size_t i = 0; i < mNumberOfLocations; ++i)
    {
        std::stringstream ss;
        ss << "l" << i;
        base::Point point;
        point.x() = rand() % static_cast<size_t>(mAreaX + 1);
        point.y() = rand() % static_cast<size_t>(mAreaY + 1);
        point.z() = rand() % static_cast<size_t>(mAreaZ + 1);
        Location::Ptr location = symbols::constants::Location::create(ss.str(), point);
        locations.push_back(location);
    }

    using namespace solvers::temporal::point_algebra;
    solvers::temporal::point_algebra::TimePointList timepoints;
    for(size_t t = 0; t < mNumberOfTimepoints; ++t)
    {
        std::stringstream ss;
        ss << "t" << t;
        TimePoint::Ptr timepoint = TimePoint::create(ss.str());
        timepoints.push_back(timepoint);
    }

    mAgentPool.clear();
    for(std::pair<owlapi::model::IRI, size_t> p : mMinPool)
    {
        size_t max = mMaxPool[p.first];
        size_t min = p.second;
        if(max > 0)
        {
            if(max < min)
            {
                throw std::invalid_argument("templ::benchmark::MissionGenerator::generate: max value of " + p.first.toString() + " smaller than min");
            } else {
                size_t count = min + (rand() % (max - min + 1));
                if(count != 0)
                {
                    mAgentPool[p.first] =  count;
                }
            }
        } else {
            if(min > 0)
            {
                mAgentPool[p.first] = min;
            }
        }

    }
    std::cout << mMinPool.toString() << std::endl;
    std::cout << mMaxPool.toString() << std::endl;

    std::cout << mAgentPool.toString() <<
        std::endl;
    // create role path for existing roles
    organization_model::OrganizationModelAsk ask(mpOrganizationModel, mAgentPool,
            true);
    Role::List roles = Role::toList(mAgentPool);


    // Create paths for mobile roles first
    std::map<Role, SpaceTime::Timeline> timelines;
    for(Role role : roles)
    {
        facades::Robot robot = facades::Robot::getInstance(role.getModel(), ask);
        if( robot.isMobile())
        {
            // Make sure system stay at a location for an interval
            SpaceTime::Timeline timeline;
            for(size_t t = 0; t < timepoints.size(); ++t)
            {
                Location::Ptr location ;
                if(t % 2 == 0)
                {
                    size_t locationIdx = rand() % static_cast<size_t>(locations.size());
                    location = locations[locationIdx];
                } else {
                    location = timeline.back().first;
                }
                timeline.push_back(SpaceTime::Point(location, timepoints[t]));
            }
            timelines[role] = timeline;
        }
    }

    solvers::transshipment::FlowNetwork flowNetwork(ask,
            locations,
            timepoints,
            timelines);

    // At this stage mobile systems are assigned
    SpaceTime::Network network = flowNetwork.getSpaceTimeNetwork();
    utils::Logger::Ptr logger = make_shared<utils::Logger>();

    SpaceTime::Network generatedNetwork;

    while(true)
    {
        std::cout << "Trying to route immobile" << std::endl;
        SpaceTime::Timelines timelinesWithImmobile;
        for(Role role : roles)
        {
            facades::Robot robot = facades::Robot::getInstance(role.getModel(), ask);
            if(!robot.isMobile())
            {
                SpaceTime::Timeline timeline;
                Location::PtrList candidateLocations = locations;
                for(size_t t = 0; t < timepoints.size(); ++t)
                {
                    size_t locationIdx = rand() %
                        static_cast<size_t>(candidateLocations.size());
                    Location::Ptr location = candidateLocations[locationIdx];
                    timeline.push_back(SpaceTime::Point(location,
                                    timepoints[t]));

                    Location::PtrList candidates;
                    SpaceTime::RoleInfoSpaceTimeTuple::Ptr vertex = network.tupleByKeys(location, timepoints[t]);
                    vertex->addRole(role, RoleInfo::ASSIGNED);
                    // Prepare next round
                    Edge::PtrList edges = network.getGraph()->getOutEdges(vertex);
                    if(edges.empty() && timepoints.size() -1 != t)
                    {
                        std::cout << "Restart adding immobile agent role: " <<
                            role.toString() << std::endl;
                        t = 0;
                        candidateLocations = locations;
                        continue;
                    }

                    for(const Edge::Ptr& e : edges)
                    {
                        SpaceTime::Network::value_t value = network.getValue(
                                dynamic_pointer_cast<SpaceTime::Network::tuple_t>(e->getTargetVertex()));

                        // validate if the transitions is actually possible
                        if(location == value) // trivially true for same location
                        {
                            candidates.push_back(value);
                        } else {
                            organization_model::Resource::Set moveToResource = {
                                Resource(vocabulary::OM::resolve("MoveTo")) };

                            RoleInfoWeightedEdge::Ptr roleEdge =
                                dynamic_pointer_cast<RoleInfoWeightedEdge>(e);
                            ModelPool pool = roleEdge->getModelPool({ RoleInfo::ASSIGNED,
                                    RoleInfo::AVAILABLE});

                            pool[role.getModel()] += 1;
                            organization_model::ModelPool::List coalitionStructure = ask.findFeasibleCoalitionStructure(pool,moveToResource, 1);
                            if(!coalitionStructure.empty())
                            {
                                candidates.push_back(value);
                            }
                        }
                    }
                    candidateLocations = candidates;
                }

                timelinesWithImmobile[role] = timeline;
            }
        }

        solvers::transshipment::MinCostFlow minCostFlow(ask,
                    logger,
                    locations,
                    timepoints,
                    timelinesWithImmobile,
                    timelines,
                    graph_analysis::algorithms::LPSolver::GLPK_SOLVER);

        std::vector<solvers::transshipment::Flaw> flaws = minCostFlow.run(true);
        if(flaws.empty())
        {
            generatedNetwork =
                minCostFlow.getFlowNetwork().getSpaceTimeNetwork();
            break;
        }
    }
    //generatedNetwork.save("/tmp/flow-network.gexf","gexf");
    return generatedNetwork;
}

} // end namespace benchmark
} // end namespace templ
