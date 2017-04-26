#ifndef TEMPL_AGENT_ROUTING_AGENT_ROUTING_GRAPH_HPP
#define TEMPL_AGENT_ROUTING_AGENT_ROUTING_GRAPH_HPP

#include <graph_analysis/BaseGraph.hpp>
#include <graph_analysis/algorithms/MultiCommodityVertex.hpp>
#include <graph_analysis/algorithms/MultiCommodityEdge.hpp>

#include "../../SpaceTime.hpp"
#include "AgentRoutingProblem.hpp"

namespace templ {
namespace solvers {
namespace agent_routing {

class AgentRoutingGraph : public SpaceTime::Network
{
public:
    AgentRoutingGraph(const AgentRoutingProblem& problem);

protected:
    AgentRoutingProblem mAgentRoutingProblem;
};

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_ROUTING_GRAPH_HPP
