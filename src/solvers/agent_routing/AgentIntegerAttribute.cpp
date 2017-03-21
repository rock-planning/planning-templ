#include "AgentIntegerAttribute.hpp"
#include <limits>
#include <sstream>

namespace templ {
namespace agent_routing {

AgentIntegerAttribute::AgentIntegerAttribute()
    : mId(std::numeric_limits<uint32_t>::infinity())
    , mLabel()
    , mMinValue(std::numeric_limits<uint32_t>::min())
    , mMaxValue(std::numeric_limits<uint32_t>::max())
    , mValue(std::numeric_limits<uint32_t>::quiet_NaN())
{}
AgentIntegerAttribute::AgentIntegerAttribute(uint32_t id, const std::string& label)
    : mId(id)
    , mLabel(label)
    , mMinValue(std::numeric_limits<uint32_t>::min())
    , mMaxValue(std::numeric_limits<uint32_t>::max())
    , mValue(std::numeric_limits<uint32_t>::quiet_NaN())
{
}

bool AgentIntegerAttribute::isValid() const
{
    return mId != std::numeric_limits<uint32_t>::infinity();
}

std::string AgentIntegerAttribute::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "AgentIntegerAttribute:" << std::endl;
    ss << hspace << "    id: " << mId << std::endl;
    ss << hspace << "    label: " << mLabel << std::endl;
    ss << hspace << "    minValue: " << mMinValue << std::endl;
    ss << hspace << "    maxValue: " << mMaxValue << std::endl;
    if( mValue == std::numeric_limits<uint32_t>::quiet_NaN())
    {
        ss << hspace << "    value: NaN" << std::endl;
    } else {
        ss << hspace << "    value: " << mValue << std::endl;
    }
    return ss.str();
}

std::string AgentIntegerAttribute::toString(const List& list, uint32_t indent)
{
    std::stringstream ss;
    List::const_iterator cit = list.begin();
    for(; cit != list.end(); ++cit)
    {
        const AgentIntegerAttribute& attribute = *cit;
        ss << attribute.toString(indent);
    }
    return ss.str();
}


} // end namespace agent_routing
} // end namespace templ
