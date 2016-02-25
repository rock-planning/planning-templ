#include "CapacityLink.hpp"

namespace templ {

CapacityLink::CapacityLink(const Role& role, uint32_t capacity)
    : graph_analysis::Edge()
    , mProvider(role)
    , mMaxCapacity(capacity)
{}

CapacityLink::~CapacityLink() {}

std::string CapacityLink::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    return hspace + "CapacityLink";
}

void CapacityLink::addUser(const Role& role, uint32_t capacity)
{ 
    if( mUsedCapacity.find(role) != mUsedCapacity.end())
    {
        throw std::invalid_argument("templ::CapacityLink::addUser: user '" + role.toString() + "'"
                "does already exist for this link");
    }
    uint32_t remainingCapacity = getRemainingCapacity();

    if(remainingCapacity < capacity)
    {
        std::stringstream ss;
        ss << "templ::CapacitLink::addUser: user '" << role.toString() << "'"
            << "cannot be added, since there is not enough capacity left: '" << remainingCapacity << "'";
        throw std::runtime_error(ss.str());
    }

    mUsedCapacity[role] = capacity;
}

uint32_t CapacityLink::getRemainingCapacity()
{
    uint32_t capacity = 0;
    std::map<Role, uint32_t>::const_iterator cit = mUsedCapacity.begin();
    for(; cit != mUsedCapacity.end(); ++cit)
    {
        capacity += cit->second;
    }
    return mMaxCapacity - capacity;
}

} // end namespace templ
