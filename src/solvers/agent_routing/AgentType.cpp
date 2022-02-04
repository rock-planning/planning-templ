
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <sstream>
#include <limits>

#include "AgentType.hpp"

namespace templ {
namespace solvers {
namespace agent_routing {

AgentType::AgentType()
    : mTypeId(std::numeric_limits<uint32_t>::infinity())
{}

AgentType::AgentType(const AgentTypeId& id)
    : mTypeId(id)
{}

void AgentType::addIntegerAttribute(const AgentIntegerAttribute& a)
{
    if( hasIntegerAttributeId(a.getId()) )
    {
        std::stringstream ss;
        ss << "templ::agent_routing::AgentType::addIntegerAttribute: ";
        ss << " agent integer attribute with id '" << a.getId() << "' is already registered";
        throw std::invalid_argument(ss.str());
    }

    mIntegerAttributes.push_back(a);
    std::sort(mIntegerAttributes.begin(), mIntegerAttributes.end(), [](const AgentIntegerAttribute& a, const AgentIntegerAttribute& b)
           {
               return a.getId() < b.getId();
           });
}

bool AgentType::hasIntegerAttributeId(uint32_t id) const
{
    try {
        getIntegerAttribute(id);
        return true;
    } catch(const std::invalid_argument& e)
    {
        return false;
    }
}

const AgentIntegerAttribute& AgentType::getIntegerAttribute(uint32_t id) const
{
    std::vector<AgentIntegerAttribute>::const_iterator cit;
    cit = std::find_if(mIntegerAttributes.begin(), mIntegerAttributes.end(), [id](const AgentIntegerAttribute& a)->bool
            {
                return a.getId() == id;
            });
    if(cit != mIntegerAttributes.end())
    {
        return *cit;
    } else {
        std::stringstream ss;
        ss << "templ::agent_routing::AgentType::getIntegerAttribute has no id '" << id << "'";
        throw std::invalid_argument(ss.str());
    }
}

std::string AgentType::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "AgentType:" << std::endl;
    ss << hspace << "    id: " << mTypeId << std::endl;
    ss << AgentIntegerAttribute::toString(mIntegerAttributes, indent + 4);
    return ss.str();
}

std::string AgentType::toString(const AgentType::List& list, uint32_t indent)
{
    std::stringstream ss;
    std::string hspace(indent, ' ');
    AgentType::List::const_iterator cit = list.begin();
    for(; cit != list.end(); ++cit)
    {
        ss << cit->toString(indent);
    }
    return ss.str();
}

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
