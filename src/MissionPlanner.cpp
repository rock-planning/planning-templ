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

namespace templ {

MissionPlanner::MissionPlanner(const Mission& mission, const OrganizationModel::Ptr& organizationModel)
    : mCurrentMission(mission)
    , mOrganizationModel(organizationModel)
    , mOrganizationModelAsk(mission.getOrganizationModel(),
            mission.getAvailableResources())
    , mOntologyAsk(mOrganizationModel->ontology())
    , mModelDistribution(0)
{
    mCurrentMission.setOrganizationModel(mOrganizationModel);
    mCurrentMission.prepare();
}

MissionPlanner::~MissionPlanner()
{
    delete mModelDistribution;
}

void MissionPlanner::nextModelAssignment()
{
    if(!mModelDistribution)
    {
        mModelDistribution = new ModelDistribution(mCurrentMission);
    }

    ModelDistribution* solvedDistribution = mModelDistribution->nextSolution();
    delete mModelDistribution;
    mModelDistribution = solvedDistribution;

    mModelDistributionSolution = mModelDistribution->getSolution();
}

void MissionPlanner::nextRoleAssignment()
{
    if(!mRoleDistribution )
    {
        mRoleDistribution = new RoleDistribution(mCurrentMission, mModelDistributionSolution);
    }

    RoleDistribution* solvedDistribution = mRoleDistribution->nextSolution();
    delete mRoleDistribution;
    mRoleDistribution = solvedDistribution;
    mRoleDistributionSolution = mRoleDistribution->getSolution();
}

void MissionPlanner::computeRoleTimelines()
{
    // Compute the timelines per role for one role solution
    std::map<Role, RoleTimeline> timelines = RoleTimeline::computeTimelines(mCurrentMission, mRoleDistributionSolution);

    std::map<Role, RoleTimeline>::iterator it = timelines.begin();
    for(; it != timelines.end(); ++it)
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

    BaseGraph::Ptr mSpaceTimeGraph = BaseGraph::getInstance();
    // Allow to find an existing tuple
    mTupleMap.clear();

    mCommodities = 0;
    {

        std::vector<pa::TimePoint::Ptr> timepoints = mCurrentMission.getTimepoints();
        pa::TimePointComparator tpc(mCurrentMission.getTemporalConstraintNetwork());
        std::sort(timepoints.begin(), timepoints.end(), [&tpc](const pa::TimePoint::Ptr& a, const pa::TimePoint::Ptr& b)
                {
                    if(a == b)
                    {
                        return false;
                    }

                    if( tpc.lessThan(a,b) )
                    {
                        std::cout << a->toString() << " is less than " << b->toString() << std::endl;
                        return true;
                    }
                    return false;
                }
        );

        std::vector<pa::TimePoint::Ptr>::const_iterator sit = timepoints.begin();
        for(; sit != timepoints.end(); ++sit)
        {
            std::cout << "Timepoint: " << (*sit)->toString() << std::endl;
        }

        std::vector<co::Location::Ptr> locations = mCurrentMission.getLocations();


        std::vector<co::Location::Ptr>::const_iterator lit = locations.begin();
        for(; lit != locations.end(); ++lit)
        {
            LocationTimepointTuple::Ptr lastTuple;

            std::vector<pa::TimePoint::Ptr>::const_iterator tit = timepoints.begin();
            for(; tit != timepoints.end(); ++tit)
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

        //{
        //    std::string filename = "/tmp/space-time-graph.dot";
        //    graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph);
        //    std::cout << "Written temporally expanded graph to: " << filename << std::endl;
        //    std::cout << "(e.g. view with 'xdot " << filename << "'" << ")" << std::endl;
        //}
        //{
        //    if( mission.getTemporalConstraintNetwork()->isConsistent())
        //    {
        //        std::cout << "Network is consistent" << std::endl;
        //    }
        //    std::string filename = "/tmp/mission-temporally-constrained-network.dot";
        //    graph_analysis::io::GraphIO::write(filename, mission.getTemporalConstraintNetwork()->getGraph());
        //    std::cout << "Written temporal constraint network to: " << filename << std::endl;
        //    std::cout << "(e.g. view with 'xdot " << filename << "'" << ")" << std::endl;
        //}

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

            //const std::vector<symbols::constants::Location::Ptr>& locations = roleTimeline.getLocations();
            //const std::vector<solvers::temporal::Interval>& getIntervals = roleTimeline.getIntervals();
            const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
            std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
            for(; fit != ftrs.end(); ++fit)
            {
                symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
                solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

                std::cout << "Location: " << location->toString() << " -- interval: " << interval.toString() << std::endl;

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
    std::cout << "Ran flow optimization: " << cost << std::cout;
    minCostFlow.storeResult();

    std::cout << "Update after flow optimization" << std::cout;
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

void MissionPlanner::save(const std::string& dir) const
{
    base::Time timestamp = base::Time::now();

    {
        std::string filename = dir + "/mission-min-cost-flow-network-" + timestamp.toString();
        graph_analysis::io::GraphIO::write(filename, mFlowGraph, graph_analysis::representation::GRAPHVIZ);
        graph_analysis::io::GraphIO::write(filename, mFlowGraph, graph_analysis::representation::GEXF);
        LOG_INFO_S << "Written min-cost flow network: " << filename << std::endl;
    }

    {

        std::string filename = dir + "/mission-space-time-network-" + timestamp.toString();
        graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph, graph_analysis::representation::GRAPHVIZ);
        graph_analysis::io::GraphIO::write(filename, mSpaceTimeGraph, graph_analysis::representation::GEXF);
        LOG_INFO_S << "Written space time network: " << filename << std::endl;
    }


}

void MissionPlanner::execute()
{
    nextModelAssignment();
    nextRoleAssignment();

    computeRoleTimelines();

}



} // end namespace templ
