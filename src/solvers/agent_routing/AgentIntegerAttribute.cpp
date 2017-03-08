#include "AgentIntegerAttribute.hpp"
#include <limits>

namespace templ {
namespace agent_routing {

AgentIntegerAttribute::AgentIntegerAttribute()
    : mId(std::numeric_limits<uint32_t>::infinity())
    , mLabel()
{}
AgentIntegerAttribute::AgentIntegerAttribute(uint32_t id, const std::string& label)
    : mId(id)
    , mLabel(label)
{
}

bool AgentIntegerAttribute::isValid() const
{
    return mId != std::numeric_limits<uint32_t>::infinity();
}



} // end namespace agent_routing
} // end namespace templ
