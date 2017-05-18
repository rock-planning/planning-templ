#include "RoleInfoTuple.hpp"
#include <algorithm>

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
    }    return roleSet->end() != roleSet->find(role);
}

std::set<Role> RoleInfo::getAllRoles() const
{
    std::set<Role> roles = mRoles;
    std::map<std::string, std::set<Role> >::const_iterator cit = mTaggedRoles.begin();
    for( ; cit != mTaggedRoles.end(); ++cit)
    {
        const std::set<Role>& taggedRoles = cit->second;
        roles.insert(taggedRoles.begin(), taggedRoles.end());
    }
    return roles;
}

std::string RoleInfo::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "    roles:" << std::endl;
    {
        std::set<Role>::const_iterator it = mRoles.begin();
        for(; it != mRoles.end(); ++it)
        {
            ss << hspace << "        " << it->toString() << std::endl;
        }
    }

    std::map<std::string, std::set<Role> >::const_iterator rit = mTaggedRoles.begin();
    for(; rit != mTaggedRoles.end(); ++rit)
    {
        ss << hspace << "    roles (" << rit->first << "):" << std::endl;
        const std::set<Role>& roles = rit->second;
        std::set<Role>::const_iterator it = roles.begin();
        for(; it != roles.end(); ++it)
        {
            ss << hspace << "        " << it->toString() << std::endl;
        }
    }

    return ss.str();
}

} // end namespace templ
