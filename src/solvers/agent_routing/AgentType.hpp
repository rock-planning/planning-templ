#ifndef TEMPL_AGENT_ROUTING_AGENT_TYPE_HPP
#define TEMPL_AGENT_ROUTING_AGENT_TYPE_HPP

#include <vector>
#include "AgentIntegerAttribute.hpp"

namespace templ {
namespace agent_routing {

class AgentType
{
public:
    AgentType();

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

private:

    // number of attributes
    std::vector<AgentIntegerAttribute> mIntegerAttributes;

};

} // end namespace agent_routing
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_TYPE_HPP
