#include "RoleInfo.hpp"
#include <algorithm>

#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <base-logging/Logging.hpp>


namespace templ {

std::map<RoleInfo::Tag, std::string> RoleInfo::TagTxt =
{{ RoleInfo::UNKNOWN, "UNKNOWN TAG"},
 { RoleInfo::ASSIGNED, "assigned"},
 { RoleInfo::REQUIRED, "required"},
 { RoleInfo::AVAILABLE, "available" },
 { RoleInfo::INFEASIBLE, "infeasible" }
}
 ;

std::map<RoleInfo::Attribute, std::string> RoleInfo::AttributeTxt =
{{ RoleInfo::UNKNOWN_ATTRIBUTE, "UNKNOWN ATTRIBUTE"},
 { RoleInfo::SAFETY, "safety"},
 { RoleInfo::RECONFIGURATION_COST, "reconfiguration cost"},
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
    mAllRoles.insert(role);
}

void RoleInfo::removeRole(const Role& role, const Tag& tag)
{
    removeRole(role, TagTxt[ tag ]);
}

void RoleInfo::removeRole(const Role& role, const std::string& tag)
{
    assert(!role.getName().empty());
    if(tag.empty())
    {
        if(mRoles.erase(role) != 0)
        {
            throw std::runtime_error("templ::RoleInfo::removeRole failed to remove (any) role");
        }
    } else {
        mTaggedRoles[tag].erase(role);
    }

    // mAllRoles.insert(role);
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

const std::set<Role> RoleInfo::getRoles(const std::set<Tag>& tags) const
{
    std::set<Role> roles;
    for(const Tag& t : tags)
    {
        const std::set<Role>& tagRoles = getRoles( t );
        roles.insert(tagRoles.begin(), tagRoles.end());
    }
    return roles;
}

moreorg::ModelPool RoleInfo::getModelPool(const std::set<Tag>& tags) const
{
    std::set<Role> roles = getRoles(tags);

    moreorg::ModelPool agentPool;
    for(const Role& r : roles)
    {
        agentPool[r.getModel()] += 1;
    }
    return agentPool;
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

bool RoleInfo::hasRole(const Role& role, const Tag& tag) const
{
    return hasRole(role, TagTxt[tag]);
}

std::string RoleInfo::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    if(!mRoles.empty())
    {
        ss << hspace << "    roles:" << std::endl;
        Role::TypeMap typeMap = Role::toTypeMap(mRoles);
        ss << Role::toString(typeMap, indent + 8);
    }

    std::map<std::string, std::set<Role> >::const_iterator rit = mTaggedRoles.begin();
    for(; rit != mTaggedRoles.end(); ++rit)
    {
        const std::set<Role>& roles = rit->second;
        if(!roles.empty())
        {
            ss << hspace << "    roles (" << rit->first << "):" << std::endl;
            Role::TypeMap typeMap = Role::toTypeMap(roles);
            ss << Role::toString(typeMap, indent + 8);
        }
    }
    return ss.str();
}

const Role::Set& RoleInfo::getAllRoles() const
{
    if(mAllRoles.empty() && !(mRoles.empty() && mTaggedRoles.empty()))
    {
        mAllRoles.insert(mRoles.begin(), mRoles.end());
	std::map<std::string, std::set<Role> >::const_iterator cit = mTaggedRoles.begin();
  	for( ; cit != mTaggedRoles.end(); ++cit)
        {
            const std::set<Role>& taggedRoles = cit->second;
            mAllRoles.insert(taggedRoles.begin(), taggedRoles.end());
        }
    }
    return mAllRoles;
}

RoleInfo::Status RoleInfo::getStatus(const owlapi::model::IRI& model, uint32_t id) const
{
    try {
        const Role& role = getRole(model, id);
        if(hasRole(role, REQUIRED))
        {
            if(hasRole(role, ASSIGNED))
            {
                return REQUIRED_ASSIGNED;
            } else {
                return REQUIRED_UNASSIGNED;
            }
        } else { // NOT REQUIRED
        {
            if(hasRole(role, ASSIGNED))
            {
                return NOTREQUIRED_ASSIGNED;
            }
	    if(hasRole(role, AVAILABLE))
                return NOTREQUIRED_AVAILABLE;
            }
        }
    } catch(const std::invalid_argument& e)
    {
        // role unknown
    }
    return UNKNOWN_STATUS;
}

std::set<RoleInfo::Status> RoleInfo::getStati() const
{
    std::set<RoleInfo::Status> stati;
    Role::Set roles = getAllRoles();
    for(const Role& role : roles)
    {
        stati.insert( getStatus(role) );
    }
    return stati;
}

std::set<std::string> RoleInfo::getTags(const Role& role) const
{
    std::set<std::string> tags;
    for(const std::pair<const std::string, Role::Set>& p : mTaggedRoles)
    {
        const std::string& tag = p.first;
        const Role::Set& roles = p.second;
        if(roles.end() != std::find(roles.begin(), roles.end(), role))
        {
            tags.insert(tag);
        }
    }
    return tags;
}

bool RoleInfo::hasTag(const Role& role) const
{
    std::set<std::string> tags;
    for(const std::pair<const std::string, Role::Set>& p : mTaggedRoles)
    {
        const Role::Set& roles = p.second;
        if(roles.end() != std::find(roles.begin(), roles.end(), role))
        {
            return true;
        }
    }
    return false;
}

const Role& RoleInfo::getRole(const owlapi::model::IRI& model, uint32_t id) const
{
    Role::Set::const_iterator cit;
    cit = std::find_if(mAllRoles.begin(), mAllRoles.end(), [&model,id](const Role& other)
            {
                return other.getModel() == model && other.getId() == id;
            });
    if(cit != mAllRoles.end())
    {
        return *cit;
    }

    throw std::invalid_argument("templ::RoleInfo::getRole: could not find role: " +
            model.toString() + std::to_string(id));
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

Role::List RoleInfo::getIntersection(const Role::Set& set0, const Role::Set& set1)
{
    Role::List result(set0.size() + set1.size());
    Role::List::iterator it;

    it = std::set_intersection(set0.begin(), set0.end(),
            set1.begin(),
            set1.end(), result.begin());

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
    mAllRoles.clear();
    mRoles.clear();
    mTaggedRoles.clear();
}

void RoleInfo::setAttribute(const std::string& attributeName, double value)
{
    mAttributes[attributeName] = value;
}

bool RoleInfo::hasAttribute(const std::string& attributeName) const
{
    return mAttributes.count(attributeName);
}

double RoleInfo::getAttribute(const std::string& attributeName) const
{
    std::map<std::string, double>::const_iterator cit = mAttributes.find(attributeName);
    if(cit != mAttributes.end())
    {
        return cit->second;
    }
    throw std::invalid_argument("templ::RoleInfo::getAttribute: no attribute '"
            + attributeName + "' found");
}

} // end namespace templ
