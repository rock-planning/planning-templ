#include "MissionGenerator.hpp"
#include <sstream>
#include <moreorg/OrganizationModel.hpp>
#include <moreorg/vocabularies/VRP.hpp>
#include <graph_analysis/algorithms/MultiCommodityMinCostFlow.hpp>

#include "../symbols/constants/Location.hpp"
#include <qxcfg/Configuration.hpp>
#include "../SpaceTime.hpp"
#include "../solvers/transshipment/MinCostFlow.hpp"
#include "../constraints/ModelConstraint.hpp"
#include "../DataPropertyAssignment.hpp"

using namespace moreorg;
using namespace templ::symbols;
using namespace templ::solvers::temporal::point_algebra;
using namespace templ::solvers::temporal;

namespace templ {
namespace benchmark {

owlapi::model::IRI MissionGenerator::mVRPOntology = "http://www.rock-robotics.org/2017/11/vrp#";

MissionGenerator::MissionGenerator(const std::string& configurationFile)
{
    srand (time(NULL));

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

    mTemporalConstraintsNoGaps =
        configuration.getValueAs<bool>("MissionGenerator/temporalConstraints/nogaps", false);

    mNumberOfLocations = configuration.getValueAs<size_t>("MissionGenerator/locations",0);
    mNumberOfTimepoints = configuration.getValueAs<size_t>("MissionGenerator/timepoints",0);
    owlapi::model::IRI organizationModelIRI = configuration.getValueAs<std::string>("MissionGenerator/organization_model");

    mpOrganizationModel = moreorg::OrganizationModel::getInstance(organizationModelIRI);

    moreorg::OrganizationModelAsk ask(mpOrganizationModel);
    owlapi::model::IRIList agents = ask.ontology().allSubClassesOf(moreorg::vocabulary::OM::Actor());
    owlapi::model::IRIList dataProperties =
        ask.ontology().allDataProperties();

    mMinPool.clear();
    mMaxPool.clear();

    // for each agent type -- fragment
    for(const owlapi::model::IRI& agent : agents)
    {
        std::string agentScope = "MissionGenerator/agentPool/" + agent.getFragment();
        mMinPool[agent] = configuration.getValueAs<size_t>(agentScope + "/min",0);
        mMaxPool[agent] = configuration.getValueAs<size_t>(agentScope + "/max",0);

        // Account for property overrides to customize, e.g., the VRP problem
        for(const owlapi::model::IRI& property : dataProperties)
        {
            try {
                std::string propertyValue = configuration.getValue(agentScope + "/overrides/" +
                        property.getFragment());
                if(!propertyValue.empty())
                {
                    double value;
                    std::stringstream ss(propertyValue);
                    ss >> value;

                    std::cout << "Applying overide: " << agent
                        << " data property: " << property
                        << " value: " << value
                        << std::endl;

                    DataPropertyAssignment dpa(agent, property, value);
                    mDataPropertyAssignments.push_back(dpa);
                }
            } catch(const std::invalid_argument& e)
            {
                // ignore missing keys
            }
        }
    }

    if(!mDataPropertyAssignments.empty())
    {
        DataPropertyAssignment::apply(mpOrganizationModel,
            mDataPropertyAssignments);
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
    OrganizationModel::Ptr om = make_shared<OrganizationModel>(mVRPOntology);
    Mission::Ptr mission = make_shared<Mission>(om, vrp.getName());

    solvers::temporal::TemporalConstraintNetwork::Ptr tcn = mission->getTemporalConstraintNetwork();
    solvers::temporal::point_algebra::TimePointComparator tpc(tcn);

    // idx
    // Add constants
    size_t idx = 0;
    TimePoint::Ptr endOfFirstInterval;
    moreorg::ModelPool availableResources;
    availableResources[vocabulary::VRP::Commodity()] = vrp.getTotalDemand();
    availableResources[vocabulary::VRP::Vehicle()] = vrp.getVehicles();
    mission->setAvailableResources(availableResources);

    std::map<Coord2D, constants::Location::Ptr> coordLocationMap;
    std::vector<TimePoint::Ptr> timepoints;
    std::vector<SpaceTime::SpaceIntervalTuple> spaceTimeIntervalTuples;
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

        // Creating the individual customer with the corresponding
        // demand
        mission->addResourceLocationCardinalityConstraint(location,
                from, to,
                vocabulary::VRP::Commodity(),
                vrp.getDemands()[idx-1],
                owlapi::model::OWLCardinalityRestriction::MIN);

        // Creating vehicle as provider
        mission->addResourceLocationCardinalityConstraint(location,
                from, to,
                vocabulary::VRP::Vehicle(),
                1,
                owlapi::model::OWLCardinalityRestriction::MIN);

        spaceTimeIntervalTuples.push_back(SpaceTime::SpaceIntervalTuple(location,
                    solvers::temporal::Interval(from, to, tpc)));
    }

    size_t depotIdx = 0;
    TimePoint::Ptr startFrom;
    TimePoint::Ptr startTo;

    TimePoint::Ptr endFrom;
    TimePoint::Ptr endTo;

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

            endFrom = tcn->getOrCreateTimePoint("t-end-0");
            endTo = tcn->getOrCreateTimePoint("t-end-1");

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

        // Making sure vehicles end at depot locations
        mission->addResourceLocationCardinalityConstraint(depotLocation,
                endFrom,
                endTo,
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

    // Limit the transport capacities of the vehicle type
    DataPropertyAssignment da(vocabulary::VRP::Vehicle(), vocabulary::OM::transportCapacity(), vrp.getCapacity());
    mission->addDataPropertyAssignment(da);

    for(point_algebra::TimePoint::Ptr t : timepoints)
    {
        try {
            // making sure that start depot is the first to be considered
            Constraint::Ptr constraint0 = tcn->addQualitativeConstraint(startTo, t, QualitativeTimePointConstraint::Less);
            mission->addConstraint(constraint0);
        } catch(...)
        {}

        try {
            // making sure the end depot is the last to be considered
            Constraint::Ptr constraint1 = tcn->addQualitativeConstraint(endTo, t, QualitativeTimePointConstraint::Greater);
            mission->addConstraint(constraint1);
        } catch(...)
        {}
    }

    // Limit to maximum 1 visit for each location
    for(const std::pair<Coord2D, constants::Location::Ptr>& p : coordLocationMap)
    {
        SpaceTime::SpaceIntervalTuple sit(p.second, solvers::temporal::Interval(
                    SpaceTime::getHorizonStart(),
                    SpaceTime::getHorizonEnd(),
                    tpc));

        std::vector<SpaceTime::SpaceIntervalTuple> sits = { sit };
        // One vehicle goes to one destination only
        {
            constraints::ModelConstraint::Ptr accessConstraint =
                make_shared<constraints::ModelConstraint>(
                        constraints::ModelConstraint::MAX_ACCESS,
                        vocabulary::VRP::Vehicle(),
                        sits,
                        1);

            mission->addConstraint(accessConstraint);
        }

        // One commodity goes to one destination only
        {
            constraints::ModelConstraint::Ptr accessConstraint =
                make_shared<constraints::ModelConstraint>(
                        constraints::ModelConstraint::MAX_ACCESS,
                        vocabulary::VRP::Commodity(),
                        sits,
                        1);
            mission->addConstraint(accessConstraint);
        }
    }

    // No resource of commodities
    constraints::ModelConstraint::Ptr distinctConstraint =
        make_shared<constraints::ModelConstraint>(
                constraints::ModelConstraint::ALL_DISTINCT,
                vocabulary::VRP::Commodity(),
                spaceTimeIntervalTuples
                );
    mission->addConstraint(distinctConstraint);

    return mission;
}

Mission::Ptr MissionGenerator::generate()
{
    SpaceTime::Network network = generateNetwork();
    Mission::Ptr mission = sampleFromNetwork(network);
    return mission;
}

Mission::Ptr MissionGenerator::sampleFromNetwork(const SpaceTime::Network& network)
{
    Mission::Ptr mission = make_shared<Mission>(mpOrganizationModel,
            "autogenerated-mission");
    mission->setAvailableResources(mAgentPool);
    mission->setDataPropertyAssignments(mDataPropertyAssignments);

    namespace pa = solvers::temporal::point_algebra;
    using namespace graph_analysis;
    using namespace symbols::constants;
    symbols::constants::Location::PtrList locations = network.getValues();
    solvers::temporal::point_algebra::TimePoint::PtrList timepoints = network.getTimepoints();

    for(Location::Ptr location : locations)
    {
        mission->addConstant(location);
    }

    for(size_t t = 1; t < timepoints.size(); ++t)
    {
        if(mTemporalConstraintsNoGaps)
        {
            pa::QualitativeTimePointConstraint::Ptr constraint =
                make_shared<pa::QualitativeTimePointConstraint>(timepoints[t-1], timepoints[t], QualitativeTimePointConstraint::Less);
            mission->addConstraint(constraint);

        } else {
            if(t > 1)
            {
                // implicit constraint exists for t_0 < t_1
                pa::QualitativeTimePointConstraint::Ptr constraint =
                    make_shared<pa::QualitativeTimePointConstraint>(timepoints[1], timepoints[t], QualitativeTimePointConstraint::Less);
                mission->addConstraint(constraint);
            }
        }
    }

    // Prepare starting locations
    for(size_t l = 0; l < locations.size(); ++l)
    {
        Location::Ptr location = locations[l];
        // A tuple with information about the allocation -> transform into a
        // Persistence Condition
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr tuple =
            network.tupleByKeys(location, timepoints[0]);

        moreorg::ModelPool agentPool = tuple->getModelPool({ RoleInfo::ASSIGNED, RoleInfo::AVAILABLE, RoleInfo::REQUIRED });
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
            dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(edge->getTargetVertex());

        if(fromTuple->second() == timepoints[0]
                || toTuple->second() == timepoints[0]
                || fromTuple->second() == toTuple->second())
        {
            continue;
        }

        if(fromTuple->first() != toTuple->first())
        {
            continue;
        }

        Role::Set fromRoles = fromTuple->getRoles({ RoleInfo::REQUIRED });
        Role::Set toRoles = toTuple->getRoles({ RoleInfo::REQUIRED });

        Role::Set agentPool;
        std::set_intersection(
                fromRoles.begin(), fromRoles.end(),
                toRoles.begin(), toRoles.end(),
                std::inserter(agentPool, agentPool.begin()));


        if(agentPool.empty())
        {
            continue;
        }

        RoleInfoWeightedEdge::Ptr roleInfoEdge =
            dynamic_pointer_cast<RoleInfoWeightedEdge>(edge);

        for(const Role& role : agentPool)
        {
            roleInfoEdge->addRole(role, { RoleInfo::REQUIRED });
        }

        LOG_INFO_S << "Add edge: " << fromRoles << " vs. " << toRoles
            << fromTuple->toString(4)
            << " -- "
            << toTuple->toString(4);


        // only use intervals for the same location
        relevantEdges.push_back(roleInfoEdge);
    }

    // Sample from the rest
    size_t total = relevantEdges.size()*std::min(mSamplingDensity,1.0);
    size_t minCount = total;
    while(minCount > 0 && !relevantEdges.empty())
    {
        size_t edgeIdx = relevantEdges.size() - 1;
        if(mSamplingDensity < 1.0)
        {
            edgeIdx = rand() % relevantEdges.size();
        }

        const Edge::Ptr& edge = relevantEdges[edgeIdx];
        RoleInfoWeightedEdge::Ptr stEdge =
            dynamic_pointer_cast<RoleInfoWeightedEdge>(edge);

        SpaceTime::RoleInfoSpaceTimeTuple::Ptr fromTuple =
            dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(edge->getSourceVertex());
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr toTuple =
            dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(edge->getTargetVertex());
        relevantEdges.erase(relevantEdges.begin() + edgeIdx);

        moreorg::ModelPool agentPool = stEdge->getModelPool({
                RoleInfo::ASSIGNED, RoleInfo::AVAILABLE, RoleInfo::REQUIRED });

        try {
            for(const std::pair<owlapi::model::IRI, size_t>& v : agentPool)
            {
                mission->addResourceLocationCardinalityConstraint(fromTuple->first(),
                        fromTuple->second(),
                        toTuple->second(),
                        v.first,
                        v.second,
                        owlapi::model::OWLCardinalityRestriction::MIN);

                std::cout << "Sample drawn from network: # " << minCount << "/" <<
                    total << std::endl
                    << "    " << fromTuple->first()->toString() << " " << fromTuple->second()->toString() << std::endl
                    << "    " << toTuple->first()->toString() << " " << toTuple->second()->toString() << std::endl
                    << "    " << v.first << " " << v.second << std::endl
                    << agentPool.toString(4);
            }
            --minCount;
        } catch(...)
        {
            // retry
            continue;
        }
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
    moreorg::OrganizationModelAsk ask(mpOrganizationModel, mAgentPool,
            true);
    Role::List roles = Role::toList(mAgentPool);


    // Create paths for mobile roles first
    std::map<Role, solvers::csp::RoleTimeline> roleTimelines;
    for(Role role : roles)
    {
        solvers::csp::RoleTimeline roleTimeline(role, ask);
        SpaceTime::Timeline timeline;
        if( roleTimeline.getRobot().isMobile())
        {
            // Make sure system stay at a location for an interval
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
                SpaceTime::Point stp(location, timepoints[t]);
                timeline.push_back(stp);
            }
        }
        roleTimeline.setTimeline(timeline);
        roleTimelines[role] = roleTimeline;
    }

    solvers::transshipment::FlowNetwork flowNetwork(roleTimelines,
            roleTimelines,
            locations,
            timepoints,
            ask);


    // At this stage mobile systems are assigned
    SpaceTime::Network network = flowNetwork.getSpaceTimeNetwork();
    utils::Logger::Ptr logger = make_shared<utils::Logger>();

    SpaceTime::Network generatedNetwork;

    while(true)
    {
        std::cout << "Trying to route immobile" << std::endl;
        std::map<Role, solvers::csp::RoleTimeline> roleTimelinesWithImmobile = roleTimelines;
        for(Role role : roles)
        {
            solvers::csp::RoleTimeline roleTimeline(role, ask);
            if(!roleTimeline.getRobot().isMobile())
            {
                SpaceTime::Timeline timeline;
                SpaceTime::RoleInfoSpaceTimeTuple::PtrList path;
                Location::PtrList candidateLocations;
                for(size_t t = 0; t < timepoints.size(); ++t)
                {
                    if(t == 0)
                    {
                        // prepare the (re)start
                        candidateLocations = locations;
                        timeline.clear();
                        path.clear();
                    }
                    // select a location for this timepoint
                    size_t locationIdx = rand() %
                        static_cast<size_t>(candidateLocations.size());
                    Location::Ptr location = candidateLocations[locationIdx];

                    SpaceTime::RoleInfoSpaceTimeTuple::Ptr vertex = network.tupleByKeys(location, timepoints[t]);
                    path.push_back(vertex);
                    timeline.push_back(SpaceTime::Point(location, timepoints[t]));

                    // Check if this is a dead end and restart if necessary
                    Edge::PtrList edges = network.getGraph()->getOutEdges(vertex);
                    if(edges.empty() && timepoints.size()-1 != t)
                    {
                        std::cout << "Restart adding immobile agent role: " <<
                            role.toString() << std::endl;
                        t = 0;
                        continue;
                    }

                    Location::PtrList candidates;
                    for(const Edge::Ptr& e : edges)
                    {
                        SpaceTime::Network::value_t value = network.getValue(
                                dynamic_pointer_cast<SpaceTime::Network::tuple_t>(e->getTargetVertex()));

                        // validate if the transitions is actually possible
                        // trivially true for a transition from/to the same location
                        if(location == value)
                        {
                            candidates.push_back(value);
                        } else {
                            moreorg::Resource::Set moveToResource = {
                                Resource(vocabulary::OM::resolve("MoveTo")) };

                            RoleInfoWeightedEdge::Ptr roleEdge =
                                dynamic_pointer_cast<RoleInfoWeightedEdge>(e);
                            ModelPool pool = roleEdge->getModelPool({ RoleInfo::ASSIGNED,
                                    RoleInfo::AVAILABLE});

                            pool[role.getModel()] += 1;
                            moreorg::ModelPool::List coalitionStructure = ask.findFeasibleCoalitionStructure(pool,moveToResource, 1);
                            if(!coalitionStructure.empty())
                            {
                                candidates.push_back(value);
                            }
                        }
                    }
                    candidateLocations = candidates;
                }

                // Update the vertices with this role after identification of a
                // valid path
                for(SpaceTime::RoleInfoSpaceTimeTuple::Ptr& vertex : path)
                {
                    vertex->addRole(role, RoleInfo::ASSIGNED);
                }

                roleTimeline.setTimeline(timeline);
                roleTimelinesWithImmobile[role] = roleTimeline;
            }
        }

        solvers::transshipment::MinCostFlow minCostFlow(roleTimelinesWithImmobile,
                    roleTimelinesWithImmobile,
                    locations,
                    timepoints,
                    ask,
                    logger,
                    graph_analysis::algorithms::LPSolver::CBC_SOLVER);

        try {
            std::vector<solvers::transshipment::Flaw> flaws = minCostFlow.run(true);
            if(flaws.empty())
            {
                generatedNetwork =
                    minCostFlow.getFlowNetwork().getSpaceTimeNetwork();
                break;
            }
        } catch(const std::runtime_error& e)
        {
            LOG_WARN_S << "No solution found: " << e.what();
            std::string filename = "/tmp/flow-network-failed.gexf";
            network.save("/tmp/flow-network-failed.gexf","gexf");
            std::cout << "Failed network saved: " << filename << std::endl;

            break;
        }
    }

    std::string filename = "/tmp/flow-network-validated.gexf";
    generatedNetwork.save("/tmp/flow-network-validated.gexf","gexf");
    std::cout << "Basic network generated: " << filename << std::endl;
    return generatedNetwork;
}

} // end namespace benchmark
} // end namespace templ
