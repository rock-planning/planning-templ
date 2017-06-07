#include "AgentRoutingGraph.hpp"

using namespace graph_analysis::algorithms;

namespace templ {
namespace solvers {
namespace agent_routing {

AgentRoutingGraph::AgentRoutingGraph(const AgentRoutingProblem& problem)
    : SpaceTime::Network(problem.getLocations(), problem.getTimePoints())
    , mAgentRoutingProblem(problem)
{
    uint32_t numberOfCommodities = problem.getAgents().size();

    // Set the default vertex
    MultiCommodityEdge::Ptr localTransitionEdge(new MultiCommodityEdge(numberOfCommodities));
    localTransitionEdge->setCapacityUpperBound(std::numeric_limits<uint32_t>::max());
    for(size_t i = 0; i < numberOfCommodities; ++i)
    {
        localTransitionEdge->setCommodityCapacityUpperBound(i, std::numeric_limits<uint32_t>::max());
        // Cost for remaining still
        localTransitionEdge->setCommodityCost(i, 0);
    }
    setLocalTransitionEdge(localTransitionEdge);


    //uint32_t
    //MultiCommodityVertex::Ptr new
    //mGraph->addVertex(supply);
}

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
