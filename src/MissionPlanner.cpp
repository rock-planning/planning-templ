#include "MissionPlanner.hpp"
#include <base/Logging.hpp>
#include <base/Time.hpp>

#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <limits>

#include <organization_model/vocabularies/OM.hpp>
#include <organization_model/facets/Robot.hpp>

#include <templ/solvers/GQReasoner.hpp>
#include <templ/PathConstructor.hpp>
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
    , mpGQReasoner(0)
    , mModelDistribution(0)
    , mModelDistributionSearchEngine(0)
    , mRoleDistribution(0)
    , mRoleDistributionSearchEngine(0)
    , mSpaceTimeGraph()
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
    mTemporalConstraintNetwork->save("/tmp/mission-planner-0-initial-temporal-constraint-network");

    mTimePointComparator = mTemporalConstraintNetwork;
    std::sort(mTimepoints.begin(), mTimepoints.end(), [this](const pa::TimePoint::Ptr& a, const pa::TimePoint::Ptr& b)
            {
                if(a == b)
                {
                    return false;
                }
                return mTimePointComparator.lessThan(a,b);
            }
    );

    std::vector<pa::TimePoint::Ptr>::const_iterator sit = mTimepoints.begin();
    for(; sit != mTimepoints.end(); ++sit)
    {
        LOG_WARN_S << "Timepoint: " << (*sit)->toString() << std::endl;
    }
}

bool MissionPlanner::nextTemporalConstraintNetwork()
{
    std::cout << "Next temporal constraint network" << std::endl;
    graph_analysis::BaseGraph::Ptr solution;
    if(!mpGQReasoner)
    {
        mpGQReasoner = new templ::solvers::GQReasoner("point", mTemporalConstraintNetwork->getGraph(), pa::QualitativeTimePointConstraint::Ptr(new pa::QualitativeTimePointConstraint));
        std::cout << "CHECK PRIMARY SOLUTION" << std::endl;
        solution = mpGQReasoner->getPrimarySolution();
        std::cout << "HAS PRIMARY SOLUTION" << std::endl;
    } else {
        std::cout << "CHECK NEXT SOLUTION" << std::endl;
        solution = mpGQReasoner->getNextSolution();

        std::cout << " gqr solution network: " << std::endl
            << mpGQReasoner->getCurrentSolutionString() << std::endl;
    }

    if(!solution)
    {
        std::cout << "NO solution" << std::endl;
        return false;
    } else {
        mTemporalConstraintNetwork->setGraph(solution);
        std::cout << "prepare next" << std::endl;
        prepareTemporalConstraintNetwork();
        return true;
    }
}

bool MissionPlanner::nextModelAssignment()
{
    std::cout << "Next model assignment" << std::endl;
    if(!mModelDistribution)
    {
        mModelDistribution = new ModelDistribution(mCurrentMission);
        mModelDistributionSearchEngine = new Gecode::BAB<ModelDistribution>(mModelDistribution);
    }

    ModelDistribution* solvedDistribution = mModelDistributionSearchEngine->next();
    if(solvedDistribution)
    {
        mModelDistributionSolution = solvedDistribution->getSolution();
        delete solvedDistribution;
        LOG_WARN_S << "Found model assignment: " << mModelDistributionSolution;
        return true;
    } else {
        return false;
    }
}

bool MissionPlanner::nextRoleAssignment()
{
    if(!mRoleDistribution)
    {
        mRoleDistribution = new RoleDistribution(mCurrentMission, mModelDistributionSolution);
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
        return false;
    }

}

void MissionPlanner::computeRoleTimelines()
{
    // Compute the timelines per role for one role solution
    mRoleTimelines = RoleTimeline::computeTimelines(mCurrentMission, mRoleDistributionSolution);

    std::map<Role, RoleTimeline>::iterator it = mRoleTimelines.begin();
    for(; it != mRoleTimelines.end(); ++it)
    {
        const RoleTimeline& timeline = it->second;
        LOG_DEBUG_S << timeline.toString();
    }
}

void MissionPlanner::computeTemporallyExpandedLocationNetwork()
{
    using namespace graph_analysis;
    namespace pa = templ::solvers::temporal::point_algebra;
    namespace co = templ::symbols::constants;

    // setup for new run
    mSpaceTimeGraph = BaseGraph::getInstance();
    // Allow to find an existing tuple
    mTupleMap.clear();
    // reset number of commodities
    mCommodities = 0;

    // Construction of the basic time-expanded network
    //
    // (t0,l0)    (t0,l1)
    //    |          |
    // (t1,l0)    (t1,l1)
    //
    std::vector<co::Location::Ptr>::const_iterator lit = mLocations.begin();
    for(; lit != mLocations.end(); ++lit)
    {
        LocationTimepointTuple::Ptr previousTuple;

        std::vector<pa::TimePoint::Ptr>::const_iterator tit = mTimepoints.begin();
        for(; tit != mTimepoints.end(); ++tit)
        {
            LocationTimepointTuple::Ptr ltTuplePtr(new LocationTimepointTuple(*lit, *tit));
            mSpaceTimeGraph->addVertex(ltTuplePtr);
            mTupleMap[ LocationTimePointPair(*lit, *tit) ] = ltTuplePtr;

            if(previousTuple)
            {
                WeightedEdge::Ptr edge(new WeightedEdge(previousTuple, ltTuplePtr, std::numeric_limits<WeightedEdge::value_t>::max()));
                mSpaceTimeGraph->addEdge(edge);
            }
            previousTuple = ltTuplePtr;
        }
    }

    {
        std::string filename = "/tmp/mission-planning--0-space-time-graph-basic_construct.dot";
        graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph);
        LOG_INFO_S << "Written temporally expanded graph to: " << filename;
        LOG_INFO_S << "(e.g. view with 'xdot " << filename << "'" << ")";
    }
    {
        if(mCurrentMission.getTemporalConstraintNetwork()->isConsistent())
        {
            std::cout << "Network is consistent" << std::endl;
        }
        std::string filename = "/tmp/mission-planning--1-temporally-constrained-network.dot";
        graph_analysis::io::GraphIO::write(filename, mCurrentMission.getTemporalConstraintNetwork()->getGraph());
        LOG_DEBUG_S << "Written temporal constraint network to: " << filename;
        LOG_DEBUG_S << "(e.g. view with 'xdot " << filename << "'" << ")";
    }

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

        // Check if this item is a payload -- WARNING: this is domain specific
        // TODO: infer capacity from role -- when robot is mobile / has
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
        LocationTimepointTuple::Ptr startTuple, endTuple;

        LOG_INFO_S << "Process (time-sorted) timeline: " << roleTimeline.toString();
        //const std::vector<symbols::constants::Location::Ptr>& locations = roleTimeline.getLocations();
        //const std::vector<solvers::temporal::Interval>& getIntervals = roleTimeline.getIntervals();
        const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
        std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
        for(; fit != ftrs.end(); ++fit)
        {
            symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
            solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

            LOG_WARN_S << "Location: " << location->toString() << " -- interval: " << interval.toString();

            // create tuple if it does not exist, otherwise reuse
            endTuple = mTupleMap[ LocationTimePointPair(location, interval.getFrom()) ];
            endTuple->addRole(role);

            // Find start node: Tuple of location and interval.getFrom()
            if(prevIntervalEnd)
            {
                startTuple = mTupleMap[ LocationTimePointPair(prevLocation, prevIntervalEnd) ];
                startTuple->addRole(role);

                std::vector< WeightedEdge::Ptr > edges = mSpaceTimeGraph->getEdges<WeightedEdge>(startTuple, endTuple);
                if(edges.empty())
                {
                    WeightedEdge::Ptr weightedEdge(new WeightedEdge(startTuple, endTuple, capacity));
                    mSpaceTimeGraph->addEdge(weightedEdge);
                } else if(edges.size() > 1)
                {
                    throw std::runtime_error("MissionPlanner: multiple capacity edges detected");
                } else { // one edge -- sum up capacities of mobile systems
                    WeightedEdge::Ptr& existingEdge = edges[0];
                    double existingCapacity = existingEdge->getWeight();
                    if(existingCapacity < std::numeric_limits<WeightedEdge::value_t>::max())
                    {
                        capacity += existingCapacity;
                        existingEdge->setWeight(capacity, 0 /*overall capacity*/);
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
    // uint32_t commodities --> see above: counted from existing immobile roles

    // Linking to graphs -- TODO: support bipartite graph in graph_analysis
    // Map a commodity flow vertex onto the space-time tuple
    std::map<Vertex::Ptr, Vertex::Ptr> commodityToSpace;
    // Reverse mapping of the space time tuple to the multicommodity vertex
    std::map<Vertex::Ptr, Vertex::Ptr> spaceToCommodity;

    // Creating the flow graph including the mapping
    // to the space-time graph
    //
    // Translating the graph into the mincommodity representation
    {
        // vertices
        VertexIterator::Ptr vertexIt = mSpaceTimeGraph->getVertexIterator();
        while(vertexIt->next())
        {
            LocationTimepointTuple::Ptr tuple = dynamic_pointer_cast<LocationTimepointTuple>(vertexIt->current());

            MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex(new MultiCommodityMinCostFlow::vertex_t(mCommodities));
            commodityToSpace[multicommodityVertex] = tuple;
            spaceToCommodity[tuple] = multicommodityVertex;

            mFlowGraph->addVertex(multicommodityVertex);
        }

        // edges
        // Iterator of the existing edges of the transport network
        // and set the commodities
        EdgeIterator::Ptr edgeIt = mSpaceTimeGraph->getEdgeIterator();
        while(edgeIt->next())
        {
            WeightedEdge::Ptr edge = dynamic_pointer_cast<WeightedEdge>(edgeIt->current());

            Vertex::Ptr source = edge->getSourceVertex();
            Vertex::Ptr target = edge->getTargetVertex();

            MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge(new MultiCommodityMinCostFlow::edge_t(mCommodities));
            multicommodityEdge->setSourceVertex( spaceToCommodity[source] );
            multicommodityEdge->setTargetVertex( spaceToCommodity[target] );

            double weight = edge->getWeight();
            uint32_t bound = 0;
            if(weight == std::numeric_limits<double>::max())
            {
                bound = std::numeric_limits<uint32_t>::max();
            } else {
                bound = static_cast<uint32_t>(weight);
            }

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
            if(!robot.isMobile())
            {
                commodityRoles.push_back(role);
                size_t commodityId = commodityRoles.size() - 1;

                const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
                std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
                Vertex::Ptr previous;
                for(; fit != ftrs.end(); ++fit)
                {
                    symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
                    solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

                    // Get the tuple in the graph
                    LocationTimepointTuple::Ptr currentTuple = mTupleMap[ LocationTimePointPair(location, interval.getFrom()) ];
                    currentTuple->addRole(role);
                    Vertex::Ptr vertex = spaceToCommodity[currentTuple];
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
    graph_analysis::io::GraphIO::write("/tmp/mission-planner-min-cost-flow-init.dot", mFlowGraph);
    uint32_t cost = minCostFlow.run();
    LOG_DEBUG_S << "Ran flow optimization: min cost: " << cost << std::endl;
    minCostFlow.storeResult();
    graph_analysis::io::GraphIO::write("/tmp/mission-planner-min-cost-flow-result.dot", mFlowGraph);

    LOG_DEBUG_S << "Update after flow optimization" << std::endl;
    // Update the mSpaceTimeGraph using the reverse mapping and adding
    // the corresponding (and new roles) roles
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
                    // Update vertices of mSpaceTimeGraph 
                    Vertex::Ptr sourceLocation = commodityToSpace[multicommodityEdge->getSourceVertex()];
                    Vertex::Ptr targetLocation = commodityToSpace[multicommodityEdge->getTargetVertex()];
                    assert(sourceLocation && targetLocation);

                    dynamic_pointer_cast<LocationTimepointTuple>(sourceLocation)->addRole(role);
                    dynamic_pointer_cast<LocationTimepointTuple>(targetLocation)->addRole(role);
                }
            }
        }
    } // end updating the mSpaceTimeGraph

    // Check on violations of the current network
    std::vector<ConstraintViolation> violations = minCostFlow.validateInflow();
    std::vector<ConstraintViolation>::const_iterator vit = violations.begin();
    for(; vit != violations.end(); ++vit)
    {
        const ConstraintViolation& violation = *vit;
        // Role that is involved into this violation
        const Role& affectedRole = commodityRoles[violation.getCommodity()];

        LOG_WARN_S << "Commodity flow violation: " << violation.toString();
        LOG_WARN_S << "Violation for " << affectedRole.toString() << " -- at: "
            << commodityToSpace[violation.getVertex()]->toString();

        // Check for violation types in order to add a suitable resolver
        if(violation.getType() == ConstraintViolation::TransFlow)
        {
            // Map violation back to LocationTimepointTuple
            LocationTimepointTuple::Ptr tuple = dynamic_pointer_cast<LocationTimepointTuple>(commodityToSpace[violation.getVertex()]);

            const RoleTimeline& roleTimeline = mRoleTimelines[affectedRole];
            const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
            std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
            for(; fit != ftrs.end(); ++fit)
            {
                symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
                solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

                if(location == tuple->first() && interval.getFrom() == tuple->second())
                {
                    LOG_WARN_S << "Found violation in timeline: enforce distiction on timeline for: " << affectedRole.toString()
                        << " between " << std::endl
                        << (*fit).toString() << " and " << (*(fit+1)).toString() << std::endl
                        << (*fit).getInterval(mModelDistribution).toString() << std::endl
                        << (*(fit+1)).getInterval(mModelDistribution).toString() << std::endl
                        << " timeline: " << std::endl
                        << roleTimeline.toString();

                    {
                        // More transport availability needed
                        // TODO: quantify request, e.g. for 1 more Payload
                        Resolver::Ptr functionalityRequest(new FunctionalityRequest(*fit, vocabulary::OM::resolve("TransportService")));
                        mResolvers.push_back(functionalityRequest);

                        // delta needs to be: missing count of resource type
                        assert(-violation.getDelta() > 0);
                    //    mRoleDistribution->addDistinct(*fit, *(fit+1), affectedRole.getModel(), -violation.getDelta(), mRoleDistributionSolution);
                        Resolver::Ptr roleAddDistinction(new RoleAddDistinction(*fit,*(fit+1), affectedRole.getModel(), -violation.getDelta(), mRoleDistributionSolution));
                        mResolvers.push_back(roleAddDistinction);

                    }
                }
            }
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
    base::Time timestamp = base::Time::now();

    if(mSpaceTimeGraph)
    {
        std::string filename = dir + "/mission-space-time-network-" + markerLabel + "-" + timestamp.toString();
        graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph, graph_analysis::representation::GRAPHVIZ);
        graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph, graph_analysis::representation::GEXF);
        LOG_WARN_S << "Written space time network: " << filename << std::endl;
    }

    if(mFlowGraph)
    {
        std::string filename = dir + "/mission-min-cost-flow-network-"  + markerLabel + "-"+ timestamp.toString();
        graph_analysis::io::GraphIO::write(filename, mFlowGraph, graph_analysis::representation::GRAPHVIZ);
        graph_analysis::io::GraphIO::write(filename, mFlowGraph, graph_analysis::representation::GEXF);
        LOG_WARN_S << "Written min-cost flow network: " << filename << std::endl;
    }
}

void MissionPlanner::renderPlan(const std::string& markerLabel, const std::string& dir) const
{
    LOG_WARN_S << "Render plan: " << markerLabel;
    using namespace graph_analysis::algorithms;
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
        LocationTimePointPair ltPair(location, interval.getFrom());
        std::map< LocationTimePointPair, LocationTimepointTuple::Ptr >::const_iterator cit = mTupleMap.find(ltPair);
        if(cit == mTupleMap.end())
        {
            throw std::runtime_error("templ::MissionPlanner::renderPlan: failed to find initial tuple for role " + role.toString());
        }
        LocationTimepointTuple::Ptr startTuple = cit->second;

        // use mSpaceTimeGraph, which contains information on role for each edge
        // after update from the flow graph
        // foreach role -- find starting point and follow path
        PathConstructor::Ptr pathConstructor(new PathConstructor(role));
        boost::function1<bool, graph_analysis::Edge::Ptr> skipper = boost::bind(&PathConstructor::invalidTransition, pathConstructor,_1);
        DFS dfs(mSpaceTimeGraph, pathConstructor, skipper); 
        dfs.run(startTuple);

        std::vector<graph_analysis::Vertex::Ptr> path = pathConstructor->getPath();
        std::vector<graph_analysis::Vertex::Ptr>::const_iterator pathIt = path.begin();
        std::stringstream ss;
        ss << "Path of role: '" << role.toString() << "'" << std::endl;
        for(; pathIt != path.end(); ++pathIt)
        {
            ss << "    " << (*pathIt)->toString() << std::endl;
        }
        LOG_WARN_S << ss.str();
    }
}

//std::vector<Flaw> MissionPlanner::execute(uint32_t maxIterations)
void MissionPlanner::execute(uint32_t maxIterations)
{
    std::cout << "Execution of planner started" << std::endl;
    uint32_t iteration = 0;
    while(nextTemporalConstraintNetwork() && iteration < maxIterations)
    {
        bool modelAssignment = nextModelAssignment();
        while(modelAssignment && iteration < maxIterations)
        {
            if(nextRoleAssignment())
            {
                computeRoleTimelines();
                computeTemporallyExpandedLocationNetwork();
                std::vector<graph_analysis::algorithms::ConstraintViolation> violations = computeMinCostFlow();
                if(violations.empty())
                {
                    std::stringstream ss;
                    ss << "solution-#" << iteration;
                    save(ss.str());
                    renderPlan(ss.str());
                    std::cout << "Found solution #" << iteration << std::endl;
                    ++iteration;
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
                        continue;
                    }
                }
            } else {
                std::cout << "No role assignment found" << std::endl;
               if(!mResolvers.empty())
               {
                   std::cout << "Trying to apply resolver: " << std::endl;
                   Resolver::Ptr resolver = mResolvers.back();
                   mResolvers.erase(mResolvers.end()-1);
                   if(resolver->getType() == Resolver::MODEL_DISTRIBUTION)
                   {
                       std::cout << "Applied model distribution solver" << std::endl;
                       resolver->apply(this);
                       continue;
                   }
                }
            }
            modelAssignment = nextModelAssignment();
        }
        if(!modelAssignment)
        {
            std::cout << "No model assignment found" << std::endl;
            std::cout << "Trying another temporal constraint network" << std::endl;
        }
    }
    std::cout << "Solutions found: " << iteration << std::endl;
}

} // end namespace templ
