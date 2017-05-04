#include "RoleInfoTuple.hpp"

namespace templ {

void RoleInfo::addRole(const Role& role, const std::string& tag)
{
    if(tag.empty())
    {
        mRoles.insert(role);
    } else {
        mTaggedRoles[tag].insert(role);
    }
}


const std::set<Role>& RoleInfo::getRoles(const std::string& tag) const
{
    if(tag.empty())
    {
        return mRoles;
    }
    return mTaggedRoles[tag];
}


bool RoleInfo::hasRole(const Role& role, const std::string& tag) const
{
    const std::set<Role>* roleSet = 0;
    if(tag.empty())
    {
        roleSet = &mRoles;
    } else {
        roleSet = &mTaggedRoles[tag];
    }
    return roleSet->end() != roleSet->find(role);
}

} // end namespace templ
