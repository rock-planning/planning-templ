#include "MissionPlanner.hpp"
#include <base/Logging.hpp>
#include <base/Time.hpp>

#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/algorithms/MultiCommodityMinCostFlow.hpp>
#include <limits>

#include <owlapi/Vocabulary.hpp>
#include <organization_model/facets/Robot.hpp>

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
    , mSpaceTimeGraph()
    , mFlowGraph()
{
    mCurrentMission.setOrganizationModel(mOrganizationModel);
    mCurrentMission.prepare();

    mLocations = mCurrentMission.getLocations();
    mTimePointComparator = mCurrentMission.getTemporalConstraintNetwork();
    mTimepoints = mCurrentMission.getTimepoints();
    std::sort(mTimepoints.begin(), mTimepoints.end(), [this](const pa::TimePoint::Ptr& a, const pa::TimePoint::Ptr& b)
            {
                if(a == b)
                {
                    return false;
                }

                if( mTimePointComparator.lessThan(a,b) )
                {
                    return true;
                }
                return false;
            }
    );

    std::vector<pa::TimePoint::Ptr>::const_iterator sit = mTimepoints.begin();
    for(; sit != mTimepoints.end(); ++sit)
    {
        LOG_DEBUG_S << "Timepoint: " << (*sit)->toString() << std::endl;
    }
}

MissionPlanner::~MissionPlanner()
{
    delete mModelDistribution;
    delete mModelDistributionSearchEngine;

    delete mRoleDistribution;
    delete mRoleDistributionSearchEngine;
}

bool MissionPlanner::nextModelAssignment()
{
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

    std::vector<co::Location::Ptr>::const_iterator lit = mLocations.begin();
    for(; lit != mLocations.end(); ++lit)
    {
        LocationTimepointTuple::Ptr lastTuple;

        std::vector<pa::TimePoint::Ptr>::const_iterator tit = mTimepoints.begin();
        for(; tit != mTimepoints.end(); ++tit)
        {
            LocationTimepointTuple::Ptr ltTuplePtr(new LocationTimepointTuple(*lit, *tit));
            mSpaceTimeGraph->addVertex(ltTuplePtr);
            mTupleMap[ LocationTimePointPair(*lit, *tit) ] = ltTuplePtr;

            if(lastTuple)
            {
                WeightedEdge::Ptr edge(new WeightedEdge(lastTuple, ltTuplePtr, std::numeric_limits<WeightedEdge::value_t>::max()));
                mSpaceTimeGraph->addEdge(edge);
            }
            lastTuple = ltTuplePtr;
        }
    }

    {
        std::string filename = "/tmp/space-time-graph.dot";
        graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph);
        LOG_INFO_S << "Written temporally expanded graph to: " << filename;
        LOG_INFO_S << "(e.g. view with 'xdot " << filename << "'" << ")";
    }
    {
        if( mCurrentMission.getTemporalConstraintNetwork()->isConsistent())
        {
            std::cout << "Network is consistent" << std::endl;
        }
        std::string filename = "/tmp/mission-temporally-constrained-network.dot";
        graph_analysis::io::GraphIO::write(filename, mCurrentMission.getTemporalConstraintNetwork()->getGraph());
        LOG_DEBUG_S << "Written temporal constraint network to: " << filename;
        LOG_DEBUG_S << "(e.g. view with 'xdot " << filename << "'" << ")";
    }

    // Per Role --> add capacities (in terms of capability of carrying a
    // payload)
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
        owlapi::model::IRI payloadClass = owlapi::vocabulary::OM::resolve("Payload");
        if(payloadClass == role.getModel() || mOntologyAsk.isSubClassOf(role.getModel(), owlapi::vocabulary::OM::resolve("Payload")))
        {
            LOG_DEBUG_S << "Delay handling of payload";
            ++mCommodities;
            continue;
        } else {
            LOG_DEBUG_S << "HANDLING: " << role.getModel() << " since it is not a " << payloadClass;
        }

        // infer capacity from role -- when robot is mobile / has
        // transportCapacity
        organization_model::facets::Robot robot(role.getModel(), mOrganizationModel);

        uint32_t capacity = robot.getPayloadTransportCapacity();
        LOG_DEBUG_S << "Role: " << role.toString() << std::endl
            << "    transport capacity: " << capacity << std::endl;


        namespace pa = templ::solvers::temporal::point_algebra;
        pa::TimePoint::Ptr prevIntervalEnd;
        co::Location::Ptr prevLocation;
        LocationTimepointTuple::Ptr startTuple, endTuple;

        LOG_WARN_S << "Process timeline: " << roleTimeline.toString();

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

            // Find start node: Tuple of location  and interval.getFrom()
            if(prevIntervalEnd)
            {
                startTuple = mTupleMap[ LocationTimePointPair(prevLocation, prevIntervalEnd) ];
                startTuple->addRole(role);

                std::vector< WeightedEdge::Ptr > edges = mSpaceTimeGraph->getEdges<WeightedEdge>(startTuple, endTuple);
                if(edges.empty())
                {
                    WeightedEdge::Ptr weightedEdge(new WeightedEdge(startTuple, endTuple, capacity));
                    mSpaceTimeGraph->addEdge(weightedEdge);
                } else {
                    if(edges.size() > 1)
                    {
                        throw std::runtime_error("MissionPlanner: multiple capacity edges detected");
                    }

                    WeightedEdge::Ptr& existingEdge = edges[0];
                    double existingCapacity = existingEdge->getWeight();
                    if(existingCapacity < std::numeric_limits<WeightedEdge::value_t>::max())
                    {
                        capacity += existingCapacity;
                        existingEdge->setWeight(capacity, 0);
                    }
                }
            }

            prevIntervalEnd = interval.getTo();
            prevLocation = location;
        }
    }

}

void MissionPlanner::computeMinCostFlow()
{
    using namespace graph_analysis;
    using namespace graph_analysis::algorithms;

    mFlowGraph = BaseGraph::getInstance();
    // uint32_t commodities --> see above: counted from existing payload
    // roles
    uint32_t edgeCapacityUpperBound = 60;

    std::map<Vertex::Ptr, Vertex::Ptr> commodityToSpace;
    std::map<Vertex::Ptr, Vertex::Ptr> spaceToCommodity;

    VertexIterator::Ptr vertexIt = mSpaceTimeGraph->getVertexIterator();
    while(vertexIt->next())
    {
        LocationTimepointTuple::Ptr tuple = boost::dynamic_pointer_cast<LocationTimepointTuple>(vertexIt->current());

        MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex(new MultiCommodityMinCostFlow::vertex_t(mCommodities));
        commodityToSpace[multicommodityVertex] = tuple;
        spaceToCommodity[tuple] = multicommodityVertex;

        mFlowGraph->addVertex(multicommodityVertex);
    }


    {
        EdgeIterator::Ptr edgeIt = mSpaceTimeGraph->getEdgeIterator();
        while(edgeIt->next())
        {
            WeightedEdge::Ptr edge = boost::dynamic_pointer_cast<WeightedEdge>(edgeIt->current());

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

    std::map<Role, RoleTimeline>::const_iterator rit = mRoleTimelines.begin();
    for(; rit != mRoleTimelines.end(); ++rit)
    {
        const Role& role = rit->first;
        const RoleTimeline& roleTimeline = rit->second;

        owlapi::model::IRI payloadClass = owlapi::vocabulary::OM::resolve("Payload");
        if(payloadClass == role.getModel() || mOntologyAsk.isSubClassOf(role.getModel(), owlapi::vocabulary::OM::resolve("Payload")))
        {
            std::cout << "Delay handling of payload" << std::endl;
        } else {
            // no payload -- no need
            continue;
        }

        commodityRoles.push_back(role);
        size_t commodityId = commodityRoles.size() - 1;

        const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
        std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
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
                boost::dynamic_pointer_cast<MultiCommodityMinCostFlow::vertex_t>(vertex);

            if(fit == ftrs.begin())
            {
                multicommodityVertex->setCommoditySupply(commodityId, 1);
            } else {
                multicommodityVertex->setCommoditySupply(commodityId, -1);
            }
        }

        // First entry can be interpreted as source
        // following entries as demands
    }

    MultiCommodityMinCostFlow minCostFlow(mFlowGraph, mCommodities);
    uint32_t cost = minCostFlow.run();
    LOG_DEBUG_S << "Ran flow optimization: min cost: " << cost << std::endl;
    minCostFlow.storeResult();

    LOG_DEBUG_S << "Update after flow optimization" << std::endl;
    {
        // Update commodites after flow optimization
        EdgeIterator::Ptr edgeIt = mFlowGraph->getEdgeIterator();
        while(edgeIt->next())
        {
            MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge =
                boost::dynamic_pointer_cast<MultiCommodityMinCostFlow::edge_t>(edgeIt->current());

            for(int i = 0; i < commodityRoles.size(); ++i)
            {
                uint32_t flow = multicommodityEdge->getCommodityFlow(i);
                if(flow > 0)
                {

                    const Role& role = commodityRoles[i];
                    Vertex::Ptr sourceLocation = commodityToSpace[multicommodityEdge->getSourceVertex()];
                    Vertex::Ptr targetLocation = commodityToSpace[multicommodityEdge->getTargetVertex()];
                    assert(sourceLocation && targetLocation);

                    boost::dynamic_pointer_cast<LocationTimepointTuple>(sourceLocation)->addRole(role);
                    boost::dynamic_pointer_cast<LocationTimepointTuple>(targetLocation)->addRole(role);
                }
            }
        }
    }
}

void MissionPlanner::save(const std::string& markerLabel, const std::string& dir) const
{
    base::Time timestamp = base::Time::now();

    if(mSpaceTimeGraph)
    {
        std::string filename = dir + "/mission-space-time-network-" + markerLabel + "-" + timestamp.toString();
        graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph, graph_analysis::representation::GRAPHVIZ);
        //graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph, graph_analysis::representation::GEXF);
        LOG_WARN_S << "Written space time network: " << filename << std::endl;
    }

    if(mFlowGraph)
    {
        std::string filename = dir + "/mission-min-cost-flow-network-"  + markerLabel + "-"+ timestamp.toString();
        graph_analysis::io::GraphIO::write(filename, mFlowGraph, graph_analysis::representation::GRAPHVIZ);
        //graph_analysis::io::GraphIO::write(filename, mFlowGraph, graph_analysis::representation::GEXF);
        LOG_WARN_S << "Written min-cost flow network: " << filename << std::endl;
    } 
}

void MissionPlanner::execute(uint32_t maxIterations)
{
    std::cout << "Execution of planner started" << std::endl;
    uint32_t iteration = 0;
    while(nextModelAssignment() && iteration < maxIterations)
    {
        if(nextRoleAssignment())
        {
            computeRoleTimelines();
            computeTemporallyExpandedLocationNetwork();
            computeMinCostFlow();

            std::stringstream ss;
            ss << "solution-#" << iteration;
            save(ss.str());
            ++iteration;

        } else {
            continue;
        }
    }
    std::cout << "Solutions found: " << iteration << std::endl;
}



} // end namespace templ
