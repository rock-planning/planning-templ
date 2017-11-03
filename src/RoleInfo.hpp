#ifndef TEMPL_ROLE_INFO_HPP
#define TEMPL_ROLE_INFO_HPP

#include <set>
#include "SharedPtr.hpp"
#include "Role.hpp"

namespace templ {

/**
 * Allow to collect related roles into a single object
 */
class RoleInfo
{
public:
    typedef shared_ptr<RoleInfo> Ptr;

    enum Tag { UNKNOWN, ASSIGNED = 1, REQUIRED, AVAILABLE};
    static std::map<Tag, std::string> TagTxt;

    RoleInfo();

    void addRole(const Role& role, const Tag& tag);

    void addRole(const Role& role, const std::string& tag = "");

    bool hasRole(const Role& role, const std::string& tag = "") const;

    const std::set<Role>& getRoles(const std::string& tag ="") const;

    const std::set<Role>& getRoles(const Tag& tag) const;

    std::set<Role> getAllRoles() const;

    /**
     * Compute the complement C between two role sets with respect to the tag0 set,
     * defined as:
     *
     \f[
        C = TAG0 \ TAG1
     \f]
     * \return complement
     */
    Role::List getRelativeComplement(const std::string& tag0, const std::string& tag1) const;

    /**
     * Get intersection between tag0 and tag1 set
     */
    Role::List getIntersection(const std::string& tag0, const std::string& tag1) const;

    /**
     * Get the symmetric difference between tag0 and tag1 set
     */
    Role::List getSymmetricDifference(const std::string& tag0, const std::string& tag1) const;

    void clear();

    std::string toString(uint32_t indent = 0) const;

protected:
    std::set<Role> mRoles;
    mutable std::map<std::string, std::set<Role> > mTaggedRoles;
};

}
#endif // TEMPL_ROLE_INFO_HPP
