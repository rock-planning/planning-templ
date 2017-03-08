#ifndef TEMPL_AGENT_ROUTING_AGENT_ROUTING_PROBLEM_HPP
#define TEMPL_AGENT_ROUTING_AGENT_ROUTING_PROBLEM_HPP

#include <vector>
#include "AgentIntegerAttribute.hpp"
#include "AgentInstanceRequirements.hpp"
#include "AgentType.hpp"

namespace templ {
namespace agent_routing {

class AgentRoutingProblem
{
public:
    AgentRoutingProblem();

    /**
     * Register an Integer Attribute
     *
     * \throws std::invalid_argument if attribute with same id is already
     * registered
     */
    void addIntegerAttribute(const AgentIntegerAttribute& a);

    /**
     *  Test if an integer attribute with the given id is already registered
     *  \return True if a attribute with given id is registered, False otherwise
     */
    bool hasIntegerAttributeId(uint32_t id) const;

    /**
     * Register an agent instance requirement
     */
    void addAgentInstanceRequirement(const AgentInstanceRequirement& r);

    /**
     * Register a general agent type
     */
    void addAgentType(const AgentType& type);


private:
    std::vector<AgentType> mAgentTypes;
    std::vector<AgentIntegerAttribute> mIntegerAttributes;

    std::vector<AgentInstanceRequirement> mAgentInstanceRequirements;

};

} // end namespace agent_routing
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_PROBLEM_HPP
