#ifndef TEMPL_AGENT_ROUTING_AGENT_TYPE_HPP
#define TEMPL_AGENT_ROUTING_AGENT_TYPE_HPP

#include <vector>
#include "AgentIntegerAttribute.hpp"

namespace templ {
namespace solvers {
namespace agent_routing {

typedef uint32_t AgentTypeId;

class AgentType
{
public:
    typedef std::vector<AgentType> List;

    AgentType();

    AgentType(const AgentTypeId& id);

    const AgentTypeId& getTypeId() const { return mTypeId; }

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

    const AgentIntegerAttribute& getIntegerAttribute(uint32_t id) const;

    /**
     * Retrieve the integer attributes
     */
    const AgentIntegerAttribute::List& getIntegerAttributes() const { return mIntegerAttributes; }

    std::string toString(uint32_t indent = 0) const;

    static std::string toString(const List& list, uint32_t = 0);

private:
    uint32_t mTypeId;

    // number of attributes
    AgentIntegerAttribute::List mIntegerAttributes;

};

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_TYPE_HPP
