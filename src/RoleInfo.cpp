#include "RoleInfoTuple.hpp"
#include <algorithm>

#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <base-logging/Logging.hpp>


namespace templ {

std::map<RoleInfo::Tag, std::string> RoleInfo::TagTxt =
{{ RoleInfo::UNKNOWN, "UNKNOWN"},
 { RoleInfo::ASSIGNED, "assigned"}
}
 ;

RoleInfo::RoleInfo()
    : mRoles()
    , mTaggedRoles()
{}

void RoleInfo::addRole(const Role& role, const Tag& tag)
{
    addRole(role, TagTxt[ tag ]);
}

void RoleInfo::addRole(const Role& role, const std::string& tag)
{
    assert(!role.getName().empty());
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

const std::set<Role>& RoleInfo::getRoles(const Tag& tag) const
{
    return getRoles( TagTxt[tag] );
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
    if(!mRoles.empty())
    {
        ss << hspace << "    roles:" << std::endl;
        {
            std::set<Role>::const_iterator it = mRoles.begin();
            for(; it != mRoles.end(); ++it)
            {
                ss << hspace << "        " << it->toString() << std::endl;
            }
        }
    }

    std::map<std::string, std::set<Role> >::const_iterator rit = mTaggedRoles.begin();
    for(; rit != mTaggedRoles.end(); ++rit)
    {
        const std::set<Role>& roles = rit->second;
        if(!roles.empty())
        {
            ss << hspace << "    roles (" << rit->first << "):" << std::endl;
            std::set<Role>::const_iterator it = roles.begin();
            for(; it != roles.end(); ++it)
            {
                ss << hspace << "        " << it->toString() << std::endl;
            }
        }
    }


    return ss.str();
}


Role::List RoleInfo::getRelativeComplement(const std::string& tag0, const std::string& tag1) const
{
    std::set<Role> tag0Roles = getRoles(tag0);
    std::set<Role> tag1Roles = getRoles(tag1);
    std::vector<Role> delta(tag0Roles.begin(), tag0Roles.end());

    for(size_t r0 = 0; r0 < delta.size();)
    {
        bool found = false;
        for(const Role& tag1Role : tag1Roles)
        {
            if( delta[r0] == tag1Role)
            {
                delta.erase( delta.begin() + r0);
                found = true;
                break;
            }
        }
        if(!found)
        {
            ++r0;
        }
    }

    return delta;
}


Role::List RoleInfo::getIntersection(const std::string& tag0, const std::string& tag1) const
{

    std::set<Role> tag0Roles = getRoles(tag0);
    std::set<Role> tag1Roles = getRoles(tag1);

    Role::List result(tag0Roles.size() + tag1Roles.size());
    Role::List::iterator it;

    it = std::set_intersection(tag0Roles.begin(), tag0Roles.end(),
            tag1Roles.begin(),
            tag1Roles.end(), result.begin());

    result.resize(it - result.begin());
    return result;
}

Role::List RoleInfo::getSymmetricDifference(const std::string& tag0, const std::string& tag1) const
{

    std::set<Role> tag0Roles = getRoles(tag0);
    std::set<Role> tag1Roles = getRoles(tag1);

    Role::List result(tag0Roles.size() + tag1Roles.size());
    Role::List::iterator it;

    it = std::set_symmetric_difference(tag0Roles.begin(), tag0Roles.end(),
            tag1Roles.begin(),
            tag1Roles.end(), result.begin());

    result.resize(it - result.begin());
    return result;
}

void RoleInfo::clear()
{
    mRoles.clear();
    mTaggedRoles.clear();
}

} // end namespace templ
