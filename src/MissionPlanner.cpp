#include "MissionPlanner.hpp"
#include <base/Logging.hpp>
#include <base/Time.hpp>

#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/SharedPtr.hpp>
#include <limits>

#include <organization_model/vocabularies/OM.hpp>
#include <organization_model/facets/Robot.hpp>

#include <templ/solvers/GQReasoner.hpp>
#include <templ/PathConstructor.hpp>
#include <templ/Plan.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <sstream>

using namespace organization_model;
using namespace templ::solvers::csp;
namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {

MissionPlanner::MissionPlanner(const Mission& mission, const OrganizationModel::Ptr& organizationModel)
    : mCurrentMission(mission)
    , mOrganizationModel(organizationModel)
    , mOrganizationModelAsk(mission.getOrganizationModel(),
            mission.getAvailableResources(), /*functional saturation bound*/ true)
    , mOntologyAsk(mOrganizationModel->ontology())
    , mModelDistribution(0)
    , mModelDistributionSearchEngine(0)
    , mRoleDistribution(0)
    , mRoleDistributionSearchEngine(0)
    , mpGQReasoner(0)
    , mpSpaceTimeNetwork(0)
    , mFlowGraph()
{
    mCurrentMission.setOrganizationModel(mOrganizationModel);
    mCurrentMission.prepare();

    mLocations = mCurrentMission.getLocations();
    mTimepoints = mCurrentMission.getTimepoints();

    // the potentially incomplete temporal constraint network
    mTemporalConstraintNetwork = mCurrentMission.getTemporalConstraintNetwork();

}

MissionPlanner::~MissionPlanner()
{
    delete mpGQReasoner;

    delete mModelDistribution;
    delete mModelDistributionSearchEngine;

    delete mRoleDistribution;
    delete mRoleDistributionSearchEngine;
}

void MissionPlanner::prepareTemporalConstraintNetwork()
{

    std::string filename = mLogging.filename("mission-planner-0-initial-temporal-constraint-network");
    mTemporalConstraintNetwork->save(filename);

    mTimePointComparator = mTemporalConstraintNetwork;
    mTimePointComparator.sort(mTimepoints);
    LOG_INFO_S << pa::TimePoint::toString(mTimepoints);
}

bool MissionPlanner::nextTemporalConstraintNetwork()
{
    graph_analysis::BaseGraph::Ptr solution;
    if(!mpGQReasoner)
    {
        mpGQReasoner = new templ::solvers::GQReasoner("point", mTemporalConstraintNetwork->getGraph(), pa::QualitativeTimePointConstraint::Ptr(new pa::QualitativeTimePointConstraint));
        solution = mpGQReasoner->getPrimarySolution();
    } else {
        solution = mpGQReasoner->getNextSolution();

        LOG_DEBUG_S << " gqr solution network: " << std::endl
            << mpGQReasoner->getCurrentSolutionString() << std::endl;
    }

    if(!solution)
    {
        LOG_DEBUG_S << "No solution exists for temporal constraint network";
        return false;
    } else {
        mTemporalConstraintNetwork->setGraph(solution);
        prepareTemporalConstraintNetwork();
        return true;
    }
}

bool MissionPlanner::nextModelAssignment()
{
    // trigger new role distribution computation
    delete mRoleDistribution;
    mRoleDistribution = NULL;

    std::cout << "Model stack: " << mModelDistributions.size() << std::endl;

    if(!mModelDistribution)
    {
        if(!mModelDistributions.empty())
        {
            std::cout << "RESET TO OLD MISSION" << std::endl;
            mCurrentMission = mConstrainedMissions.back();
            mConstrainedMissions.pop_back(); 

            mModelDistribution = mModelDistributions.back();
            mModelDistributions.pop_back();

            mModelDistributionSearchEngine = mModelDistributionSearchStates.back();
            mModelDistributionSearchStates.pop_back();
        } else {
            std::cout << "RESET TO NEW MISSION" << std::endl;
            mModelDistribution = new ModelDistribution(mCurrentMission);
            mModelDistributionSearchEngine = new Gecode::BAB<ModelDistribution>(mModelDistribution);
        }
    }

    ModelDistribution* solvedDistribution = mModelDistributionSearchEngine->next();
    if(solvedDistribution)
    {
        mModelDistributionSolution = solvedDistribution->getSolution();
        delete solvedDistribution;
        LOG_WARN_S << "Found model assignment: " << mModelDistributionSolution;
        return true;
    } else {
        delete mModelDistributionSearchEngine;
        mModelDistributionSearchEngine = NULL;
        delete mModelDistribution;
        mModelDistribution = NULL;

        return mModelDistributionSearchStates.empty();
    }
}

bool MissionPlanner::nextRoleAssignment()
{
    if(!mRoleDistribution)
    {
        std::cout << "NEW ROLE ASSIGNMENT" << std::endl;
        mRoleDistribution = new RoleDistribution(mCurrentMission, mModelDistributionSolution);
        delete mRoleDistributionSearchEngine;
        mRoleDistributionSearchEngine = new Gecode::BAB<RoleDistribution>(mRoleDistribution);
    }

    RoleDistribution* solvedDistribution = mRoleDistributionSearchEngine->next();
    if(solvedDistribution)
    {
        mRoleDistributionSolution = solvedDistribution->getSolution();
        delete solvedDistribution;
        LOG_WARN_S << "Found role assignment: " << mRoleDistributionSolution;
        return true;
    } else {
        LOG_WARN_S << "NO NEXT ROLE_ASSIGNMENT";
        return false;
    }

}

void MissionPlanner::computeRoleTimelines()
{
    // Compute the timelines per role for one role solution
    mRoleTimelines = RoleTimeline::computeTimelines(mCurrentMission, mRoleDistributionSolution);
    LOG_DEBUG_S << RoleTimeline::toString(mRoleTimelines);
}

void MissionPlanner::computeTemporallyExpandedLocationNetwork()
{
    using namespace graph_analysis;
    namespace pa = templ::solvers::temporal::point_algebra;
    namespace co = templ::symbols::constants;

    if(mpSpaceTimeNetwork)
    {
        delete mpSpaceTimeNetwork;
    }

    mpSpaceTimeNetwork = new TemporallyExpandedNetwork<co::Location>(mLocations, mTimepoints);

    {
        std::string filename = mLogging.filename("computeTemporallyExpandedNetwork-0-space-time-graph-basic_construct.dot");
        graph_analysis::io::GraphIO::write(filename, mpSpaceTimeNetwork->getGraph());
        LOG_INFO_S << "Written temporally expanded graph to: " << filename;
        LOG_INFO_S << "(e.g. view with 'xdot " << filename << "'" << ")";
    }
    {
        if(mCurrentMission.getTemporalConstraintNetwork()->isConsistent())
        {
            std::cout << "Network is consistent" << std::endl;
        }
        std::string filename = mLogging.filename("computeTemporallyExpandedNetwork-1-temporally-constrained-network.dot");
        graph_analysis::io::GraphIO::write(filename, mCurrentMission.getTemporalConstraintNetwork()->getGraph());
        LOG_DEBUG_S << "Written temporal constraint network to: " << filename;
        LOG_DEBUG_S << "(e.g. view with 'xdot " << filename << "'" << ")";
    }


    // reset number of commodities
    // where a commodity represent a resource that need to be routed along the
    // transport network
    // here: commodity == immobile robotic agent
    mCommodities = 0;
    // -----------------------------------------
    // Add capacity-weighted edges to the graph
    // -----------------------------------------
    // Per Role --> add capacities (in terms of capability of carrying an immobile system)
    std::map<Role, RoleTimeline>::const_iterator rit = mRoleTimelines.begin();
    for(; rit != mRoleTimelines.end(); ++rit)
    {
        // infer connections from timeline
        // sequentially ordered timeline
        // locations and timeline
        // connection from (l0, i0_end) ---> (l1, i1_start)
        //
        const Role& role = rit->first;
        RoleTimeline roleTimeline = rit->second;
        roleTimeline.sortByTime();

        // Check if this item is mobile, i.e. change change the location 
        // WARNING: this is domain specific
        //
        // transportCapacity
        organization_model::facets::Robot robot(role.getModel(), mOrganizationModelAsk);
        if(!robot.isMobile())
        {
            LOG_DEBUG_S << "Delay handling of immobile system: " << role.getModel();
            ++mCommodities;
            continue;
        } else {
            LOG_DEBUG_S << "Add capacity for mobile system: " << role.getModel();
        }

        uint32_t capacity = robot.getPayloadTransportCapacity();
        LOG_DEBUG_S << "Role: " << role.toString() << std::endl
            << "    transport capacity: " << capacity << std::endl;

        namespace pa = templ::solvers::temporal::point_algebra;
        pa::TimePoint::Ptr prevIntervalEnd;
        co::Location::Ptr prevLocation;
        SpaceTimeNetwork::tuple_t::Ptr startTuple, endTuple;

        LOG_INFO_S << "Process (time-sorted) timeline: " << roleTimeline.toString();
        const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
        std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
        for(; fit != ftrs.end(); ++fit)
        {
            symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
            solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

            LOG_WARN_S << "Location: " << location->toString() << " -- interval: " << interval.toString();

            // create tuple if it does not exist?
            endTuple = mpSpaceTimeNetwork->tupleByKeys(location, interval.getFrom());
            endTuple->addRole(role);

            // Find start node: Tuple of location and interval.getFrom()
            if(prevIntervalEnd)
            {
                startTuple = mpSpaceTimeNetwork->tupleByKeys(prevLocation, prevIntervalEnd);
                startTuple->addRole(role);

                std::vector< WeightedEdge::Ptr > edges = mpSpaceTimeNetwork->getGraph()->getEdges<WeightedEdge>(startTuple, endTuple);
                if(edges.empty())
                {
                    WeightedEdge::Ptr weightedEdge(new WeightedEdge(startTuple, endTuple, capacity));
                    mpSpaceTimeNetwork->getGraph()->addEdge(weightedEdge);
                } else if(edges.size() > 1)
                {
                    throw std::runtime_error("MissionPlanner: multiple capacity edges detected");
                } else { // one edge -- sum up capacities of mobile systems
                    WeightedEdge::Ptr& existingEdge = edges[0];
                    double existingCapacity = existingEdge->getWeight();
                    if(existingCapacity < std::numeric_limits<WeightedEdge::value_t>::max())
                    {
                        capacity += existingCapacity;
                        existingEdge->setWeight(capacity, 0 /*index of 'overall capacity'*/);
                    }
                }
            }

            prevIntervalEnd = interval.getTo();
            prevLocation = location;
        }
    }
}

std::vector<graph_analysis::algorithms::ConstraintViolation> MissionPlanner::computeMinCostFlow()
{
    using namespace graph_analysis;
    using namespace graph_analysis::algorithms;

    mFlowGraph = BaseGraph::getInstance();

    graph_analysis::BipartiteGraph bipartiteGraph(mFlowGraph, mpSpaceTimeNetwork->getGraph());

    // uint32_t commodities --> see above: counted from existing immobile roles

    // Creating the flow graph including the mapping
    // to the space-time graph

    // Translating the graph into the mincommodity representation
    {
        // vertices
        VertexIterator::Ptr vertexIt = mpSpaceTimeNetwork->getGraph()->getVertexIterator();
        while(vertexIt->next())
        {
            MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex(new MultiCommodityMinCostFlow::vertex_t(mCommodities));
            bipartiteGraph.addMapping(multicommodityVertex, vertexIt->current());
        }

        // edges
        // Iterator of the existing edges of the transport network
        // and set the commodities
        EdgeIterator::Ptr edgeIt = mpSpaceTimeNetwork->getGraph()->getEdgeIterator();
        while(edgeIt->next())
        {
            WeightedEdge::Ptr edge = dynamic_pointer_cast<WeightedEdge>(edgeIt->current());

            Vertex::Ptr source = edge->getSourceVertex();
            Vertex::Ptr target = edge->getTargetVertex();

            // Create an edge in the multicommodity representation that correspond to the space time graph
            MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge(new MultiCommodityMinCostFlow::edge_t(mCommodities));
            multicommodityEdge->setSourceVertex( bipartiteGraph.getUniquePartner(source) );
            multicommodityEdge->setTargetVertex( bipartiteGraph.getUniquePartner(target) );

            double weight = edge->getWeight();
            uint32_t bound = 0;
            if(weight == std::numeric_limits<double>::max())
            {
                bound = std::numeric_limits<uint32_t>::max();
            } else {
                bound = static_cast<uint32_t>(weight);
            }

            // Upper bound is the maximum edge capacity for a commodity
            multicommodityEdge->setCapacityUpperBound(bound);
            for(size_t i = 0; i < mCommodities; ++i)
            {
                multicommodityEdge->setCommodityCapacityUpperBound(i, bound);
            }
            mFlowGraph->addEdge(multicommodityEdge);
        }
    }


    std::vector<Role> commodityRoles;
    // Dealing with requirements from immobile systems
    // set 'start','end' and 'waypoints' for the immobile systems
    {
        std::map<Role, RoleTimeline>::const_iterator rit = mRoleTimelines.begin();
        for(; rit != mRoleTimelines.end(); ++rit)
        {
            const Role& role = rit->first;
            const RoleTimeline& roleTimeline = rit->second;

            organization_model::facets::Robot robot(role.getModel(), mOrganizationModelAsk);
            if(!robot.isMobile()) // only immobile systems are relevant
            {
                // Allow to later map roles back from index
                commodityRoles.push_back(role);
                size_t commodityId = commodityRoles.size() - 1;

                // Retrieve the sorted list of FluentTimeResources
                const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
                std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
                Vertex::Ptr previous;
                for(; fit != ftrs.end(); ++fit)
                {
                    // Map back space and time to human-readable information
                    symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
                    solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

                    // Get the tuple in the graph and augment with role
                    // information
                    SpaceTimeNetwork::tuple_t::Ptr currentTuple = mpSpaceTimeNetwork->tupleByKeys(location, interval.getFrom());
                    currentTuple->addRole(role);

                    Vertex::Ptr vertex = bipartiteGraph.getUniquePartner(currentTuple);
                    assert(vertex);
                    MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex =
                        dynamic_pointer_cast<MultiCommodityMinCostFlow::vertex_t>(vertex);

                    // - first entry (start point): can be interpreted as source
                    // - intermediate entries (transflow) as lower flow bound
                    // - final entry as demand
                    if(fit == ftrs.begin())
                    {
                        multicommodityVertex->setCommoditySupply(commodityId, 1);
                    } else if(fit+1 == ftrs.end())
                    {
                        multicommodityVertex->setCommoditySupply(commodityId, -1);
                    } else { // intermediate ones
                        // set minimum flow that needs to go through this node
                        // for this commodity (i.e. can be either 0 or 1)
                        multicommodityVertex->setCommodityMinTransFlow(commodityId, 1);
                    }
                    previous = vertex;
                }
            } // end if !robot.isMobile()
        } // end for role timelines
    } // end scope handling immobile units

    MultiCommodityMinCostFlow minCostFlow(mFlowGraph, mCommodities);
    {
        std::string filename  = mLogging.filename("mission-planner-min-cost-flow-init.dot");
        graph_analysis::io::GraphIO::write(filename, mFlowGraph);
    }
    uint32_t cost = minCostFlow.run();
    LOG_DEBUG_S << "Ran flow optimization: min cost: " << cost << std::endl;
    minCostFlow.storeResult();

    {
        std::string filename  = mLogging.filename("mission-planner-min-cost-flow-result.dot");
        graph_analysis::io::GraphIO::write(filename, mFlowGraph);
    }

    LOG_DEBUG_S << "Update after flow optimization" << std::endl;
    // Update the mSpaceTimeNetwork using the reverse mapping and adding
    // the corresponding (and new) roles
    // TODO: check if this should not be updated at a later state, e.g. after
    // violation processing
    {
        // Update commodities after flow optimization
        EdgeIterator::Ptr edgeIt = mFlowGraph->getEdgeIterator();
        while(edgeIt->next())
        {
            MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge =
                dynamic_pointer_cast<MultiCommodityMinCostFlow::edge_t>(edgeIt->current());

            for(size_t i = 0; i < commodityRoles.size(); ++i)
            {
                uint32_t flow = multicommodityEdge->getCommodityFlow(i);
                if(flow > 0)
                {
                    const Role& role = commodityRoles[i];
                    // Update vertices of mSpaceTimeNetwork
                    Vertex::Ptr sourceLocation = bipartiteGraph.getUniquePartner(multicommodityEdge->getSourceVertex());
                    Vertex::Ptr targetLocation = bipartiteGraph.getUniquePartner(multicommodityEdge->getTargetVertex());
                    assert(sourceLocation && targetLocation);

                    dynamic_pointer_cast<SpaceTimeNetwork::tuple_t>(sourceLocation)->addRole(role);
                    dynamic_pointer_cast<SpaceTimeNetwork::tuple_t>(targetLocation)->addRole(role);
                }
            }
        }
    } // end updating the mSpaceTimeNetwork

    // Check on violations of the current network
    std::vector<ConstraintViolation> violations = minCostFlow.validateInflow();
    LOG_WARN_S << "FOUND " << violations.size() << " violation of the commodity flow";
    // Check for violation types in order to add a suitable resolver
    mResolvers.clear();
    std::vector<ConstraintViolation>::const_iterator vit = violations.begin();
    for(; vit != violations.end(); ++vit)
    {
        const ConstraintViolation& violation = *vit;
        // Role that is involved into this violation
        const Role& affectedRole = commodityRoles[violation.getCommodity()];
        // Map violation from multicommodity vertex back to SpaceTimeNetwork tuple
        Vertex::Ptr spaceTimePartnerVertex = bipartiteGraph.getUniquePartner(violation.getVertex());
        SpaceTimeNetwork::tuple_t::Ptr tuple = dynamic_pointer_cast<SpaceTimeNetwork::tuple_t>(spaceTimePartnerVertex);

        SpaceTimeNetwork::value_t::Ptr location = dynamic_pointer_cast<SpaceTimeNetwork::value_t>(tuple->first());

        LOG_WARN_S << "Commodity flow violation: " << violation.toString();
        LOG_WARN_S << "Violation for " << affectedRole.toString() << " -- at: "
            << spaceTimePartnerVertex->toString();

        const RoleTimeline& roleTimeline = mRoleTimelines[affectedRole];
        std::vector<FluentTimeResource>::const_iterator fit;
        try {
             fit = getFluent(roleTimeline, tuple);
        } catch(const std::invalid_argument& e)
        {
            // no fix possible
            LOG_WARN_S << "NO FIX POSSIBLE";
            return violations;
        }

        switch(violation.getType())
        {
            // In the case of transflow we add a triebreaker between the current
            // and the subsequent requirement
            case ConstraintViolation::TransFlow:
            {
                FluentTimeResource ftr = *fit;
                FluentTimeResource ftr_subsequent = *(fit+1);

                std::cout << "Transflow violation in timeline: enforce distiction on timeline for: " << affectedRole.toString()
                    << " between " << std::endl
                    << ftr.toString() << " and " << ftr_subsequent.toString() << std::endl
                    << ftr.getInterval(mModelDistribution).toString() << std::endl
                    << ftr_subsequent.getInterval(mModelDistribution).toString() << std::endl
                    << " timeline: " << std::endl
                    << roleTimeline.toString()
                    << std::endl;

                organization_model::facets::Robot robot(affectedRole.getModel(), mOrganizationModelAsk);
                if(!robot.isMobile())
                {
                    // More transport availability needed
                    // TODO: quantify request, e.g. for 1 more Payload
                    Resolver::Ptr functionalityRequest(new FunctionalityRequest(location, ftr.getInterval(mModelDistribution), vocabulary::OM::resolve("TransportService")));
                    mResolvers.push_back(functionalityRequest);
                }

                 // delta needs to be: missing count of resource type
                 assert(-violation.getDelta() > 0);
                 Resolver::Ptr roleAddDistinction(new RoleAddDistinction(ftr, ftr_subsequent, affectedRole.getModel(), abs( violation.getDelta() ), mRoleDistributionSolution));
                 mResolvers.push_back(roleAddDistinction);

                break;
            }
            // In the case of minflow we add a triebreaker between the current
            // and the previous requirement
            case ConstraintViolation::MinFlow:
            {
                FluentTimeResource ftr = *fit;
                FluentTimeResource ftr_previous = *(fit-1);

                std::cout << "Minflow violation in timeline: enforce distiction on timeline for: " << affectedRole.toString()
                    << " between " << std::endl
                    << ftr_previous.toString() << " and " << ftr.toString() << std::endl
                    << ftr_previous.getInterval(mModelDistribution).toString() << std::endl
                    << ftr.getInterval(mModelDistribution).toString() << std::endl
                    << " timeline: " << std::endl
                    << roleTimeline.toString()
                    << std::endl;

                organization_model::facets::Robot robot(affectedRole.getModel(), mOrganizationModelAsk);
                if(!robot.isMobile())
                {
                    std::cout << "Robot is not mobile thus requesting a TransportService" << std::endl;
                   // More transport availability needed
                   // TODO: quantify request, e.g. for 1 more Payload
                   Resolver::Ptr functionalityRequest(new FunctionalityRequest(location, ftr.getInterval(mModelDistribution), vocabulary::OM::resolve("TransportService")));
                   mResolvers.push_back(functionalityRequest);
                }

                // delta needs to be: missing count of resource type
                assert(-violation.getDelta() > 0);
                Resolver::Ptr roleAddDistinction(new RoleAddDistinction(ftr_previous, ftr, affectedRole.getModel(), abs( violation.getDelta() ), mRoleDistributionSolution));
                mResolvers.push_back(roleAddDistinction);
            }
            break;
        }
    }
    for(size_t i = 0; i < commodityRoles.size(); ++i)
    {
        LOG_WARN_S << "#" << i << " --> " << commodityRoles[i].toString();
    }

    return violations;
}

void MissionPlanner::save(const std::string& markerLabel, const std::string& dir) const
{
    if(mpSpaceTimeNetwork)
    {
        std::string filename = "mission-space-time-network-" + markerLabel;
        if(dir.empty())
        {
            filename = mLogging.filename(filename);
        } else {
            filename = dir + "/" + filename;
        }
        graph_analysis::io::GraphIO::write(filename, mpSpaceTimeNetwork->getGraph(), graph_analysis::representation::GRAPHVIZ);
        graph_analysis::io::GraphIO::write(filename, mpSpaceTimeNetwork->getGraph(), graph_analysis::representation::GEXF);
        LOG_WARN_S << "Written space time network: " << filename << std::endl;
    }

    if(mFlowGraph)
    {
        std::string filename = "mission-min-cost-flow-network-"  + markerLabel;
        if(dir.empty())
        {
            filename = mLogging.filename(filename);
        } else {
            filename = dir + "/" + filename;
        }

        graph_analysis::io::GraphIO::write(filename, mFlowGraph, graph_analysis::representation::GRAPHVIZ);
        graph_analysis::io::GraphIO::write(filename, mFlowGraph, graph_analysis::representation::GEXF);
        LOG_WARN_S << "Written min-cost flow network: " << filename << std::endl;
    }
}

Plan MissionPlanner::renderPlan(const std::string& markerLabel) const
{
    Plan plan(markerLabel);

    std::map<Role, RoleTimeline>::const_iterator it = mRoleTimelines.begin();
    for(; it != mRoleTimelines.end(); ++it)
    {
        const Role& role = it->first;
        RoleTimeline roleTimeline = it->second;
        roleTimeline.sortByTime();
       
        // Get first space/location tuple
        const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
        symbols::constants::Location::Ptr location = roleTimeline.getLocation(ftrs.front());
        solvers::temporal::Interval interval = roleTimeline.getInterval(ftrs.front());
        
        SpaceTimeNetwork::tuple_t::Ptr startTuple;
        try {
            startTuple = mpSpaceTimeNetwork->tupleByKeys(location, interval.getFrom());
        } catch(const std::invalid_argument& e)
        {
            throw std::runtime_error("templ::MissionPlanner::renderPlan: failed to find initial tuple for role " + role.toString());
        }

        using namespace graph_analysis::algorithms;
        // use SpaceTimeNetwork, which contains information on role for each edge
        // after update from the flow graph
        // foreach role -- find starting point and follow path
        PathConstructor::Ptr pathConstructor(new PathConstructor(role));
        boost::function1<bool, graph_analysis::Edge::Ptr> skipper = boost::bind(&PathConstructor::invalidTransition, pathConstructor,_1);
        DFS dfs(mpSpaceTimeNetwork->getGraph(), pathConstructor, skipper); 
        dfs.run(startTuple);

        plan.add(role, pathConstructor->getPath());
    }
    LOG_WARN_S << "Rendered plan: " << plan.toString();
    return plan;
}

std::vector<Plan> MissionPlanner::execute(uint32_t maxIterations)
{
    std::vector<Plan> plans;
    std::cout << "Execution of planner started" << std::endl;
    uint32_t iteration = 0;
    while(nextTemporalConstraintNetwork() && iteration < maxIterations)
    {
        std::cout << "EVAL NEXT MODEL ASSIGNMENT -- TOP" << std::endl;
        bool modelAssignment = nextModelAssignment();
        while(modelAssignment && iteration < maxIterations)
        {
            std::cout << "EVAL NEXT ROLE ASSIGNMENT" << std::endl;
            if(nextRoleAssignment())
            {
                computeRoleTimelines();
                computeTemporallyExpandedLocationNetwork();
                // will modify the resolvers
                std::vector<graph_analysis::algorithms::ConstraintViolation> violations = computeMinCostFlow();
                if(violations.empty())
                {
                    std::stringstream ss;
                    ss << "solution-#" << iteration;
                    save(ss.str());
                    plans.push_back( renderPlan() );
                    std::cout << "Found solution #" << iteration << std::endl;
                    ++iteration;
                    mLogging.incrementSessionId();
                    break;
                } else if(!mResolvers.empty())
                {
                    std::cout << "Trying to apply resolver: " << std::endl;
                    Resolver::Ptr resolver = mResolvers.back();
                    mResolvers.erase(mResolvers.end()-1);
                    if(resolver->getType() == Resolver::ROLE_DISTRIBUTION)
                    {
                        std::cout << "Applied role distribution solver" << std::endl;
                        resolver->apply(this);
                        std::cout << "Press enter to continue" << std::endl;
                        std::string enter;
                        std::cin >> enter;
                        continue;
                    } else {
                        throw std::runtime_error("INVALID ASSUMPTION last one is not role distribution solver");
                    }
                }
            } else {
                std::cout << "No role assignment found" << std::endl;
                while(!mResolvers.empty())
                {
                    std::cout << "Trying to apply resolver: " << std::endl;
                    Resolver::Ptr resolver = mResolvers.back();
                    mResolvers.erase(mResolvers.end()-1);
                    if(resolver->getType() == Resolver::MODEL_DISTRIBUTION)
                    {
                        std::cout << "Applied model distribution solver" << std::endl;
                        try {
                            resolver->apply(this);
                        } catch(const std::exception& e)
                        {
                            std::cout << "Failed to apply resolver " << e.what();
                            if(!mResolvers.empty())
                            {
                                std::cout << "Trying next" << std::endl;
                            } else {
                                std::cout << "No remaining resolver";
                            }
                        }
                    } else {
                        // ignore
                    }
                 }
            }
            std::cout << "EVAL NEXT MODEL ASSIGNMENT -- BOTTOM" << std::endl;
            modelAssignment = nextModelAssignment();
        }
        if(!modelAssignment)
        {
            std::cout << "No model assignment found --trying another temporal constraint network" << std::endl;
        }
    }
    // Disable sessions to log into base dir
    mLogging.disableSessions();
    std::string filename = mLogging.filename("plans.log");
    Plan::save(plans, filename);

    filename = mLogging.filename("actionplans.log");
    Plan::saveAsActionPlan(plans, mCurrentMission, filename);

    return plans;
}

std::vector<FluentTimeResource>::const_iterator MissionPlanner::getFluent(const RoleTimeline& roleTimeline, const SpaceTimeNetwork::tuple_t::Ptr& tuple)
{
    LOG_WARN_S << "Find tuple: " << tuple->toString() << " in timeline " << roleTimeline.toString();

    const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
    std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
    for(; fit != ftrs.end(); ++fit)
    {
        symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
        solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

        if(location == tuple->first() && interval.getFrom() == tuple->second())
        {
            return fit;
        }
    }
    throw std::invalid_argument("MissionPlanner::getFluent: could not retrieve corresponding fluent in timeline: '" 
            + roleTimeline.toString() + "' from tuple '" + tuple->toString());
}

void MissionPlanner::constrainMission(const symbols::constants::Location::Ptr& location,
        const solvers::temporal::Interval& interval,
        const owlapi::model::IRI& model)
{
    std::cout << "CONSTRAINING MISSION AT:" <<
        location->toString() << "[" << interval.toString() << " with model: " << model.toString()
        << std::endl;

    LOG_WARN_S << "STORE CURRENT MISSION";
    Mission mission = mCurrentMission;
    mConstrainedMissions.push_back(mission);

    LOG_WARN_S << "add resource constraint";
    mCurrentMission.addResourceLocationCardinalityConstraint(location,
            interval.getFrom(), interval.getTo(),
            model);

    mModelDistributions.push_back(mModelDistribution);
    std::cout << "Model stack: " << mModelDistributions.size() << std::endl;
    mModelDistributionSearchStates.push_back(mModelDistributionSearchEngine);

    std::cout << "New model distribution" << std::endl;
    mModelDistribution = new ModelDistribution(mCurrentMission);
    mModelDistributionSearchEngine = new Gecode::BAB<solvers::csp::ModelDistribution>(mModelDistribution);

}

} // end namespace templ
