#ifndef TEMPL_ROLE_INFO_HPP
#define TEMPL_ROLE_INFO_HPP

#include <set>
#include "templ/Role.hpp"

namespace templ {

class RoleInfo
{
public:
    typedef shared_ptr<RoleInfo> Ptr;

    RoleInfo();

    void addRole(const Role& role, const std::string& tag = "");

    bool hasRole(const Role& role, const std::string& tag = "") const;

    const std::set<Role>& getRoles(const std::string& tag ="") const;

    std::set<Role> getAllRoles() const;

    void clear();

    std::string toString(uint32_t indent = 0) const;

protected:
    std::set<Role> mRoles;
    mutable std::map<std::string, std::set<Role> > mTaggedRoles;
};

}
#endif // TEMPL_ROLE_INFO_HPP
