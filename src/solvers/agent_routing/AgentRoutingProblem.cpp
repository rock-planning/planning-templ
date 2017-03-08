#include "AgentRoutingProblem.hpp"
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace templ {
namespace agent_routing {

AgentRoutingProblem::AgentRoutingProblem()
{}

void AgentRoutingProblem::addIntegerAttribute(const AgentIntegerAttribute& a)
{
    if( hasIntegerAttributeId(a.getId()) )
    {
        std::stringstream ss;
        ss << "templ::agent_routing::AgentRoutingProblem::addIntegerAttribute: ";
        ss << " agent integer attribute with id '" << a.getId() << "' is already registered";
        throw std::invalid_argument(ss.str());
    }

    mIntegerAttributes.push_back(a);
    std::sort(mIntegerAttributes.begin(), mIntegerAttributes.end(), [](const AgentIntegerAttribute& a, const AgentIntegerAttribute& b)
           {
               return a.getId() < b.getId();
           });
}

bool AgentRoutingProblem::hasIntegerAttributeId(uint32_t id) const
{
    std::vector<AgentIntegerAttribute>::const_iterator cit;
    cit = std::find_if(mIntegerAttributes.begin(), mIntegerAttributes.end(), [id](const AgentIntegerAttribute& a)->bool
            {
                return a.getId() == id;
            });
    return cit != mIntegerAttributes.end();
}

void AgentRoutingProblem::addAgentInstanceRequirement(const AgentInstanceRequirement& r)
{
    mAgentInstanceRequirements.push_back(r);
}

void AgentRoutingProblem::addAgentType(const AgentType& type)
{
    mAgentTypes.push_back(type);
}

} // namespace agent_routing
} // namespace templ
