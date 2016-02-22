#ifndef TEMPL_ROLE_INFO_TUPLE_HPP
#define TEMPL_ROLE_INFO_TUPLE_HPP

#include <set>
#include <templ/Role.hpp>
#include <templ/Tuple.hpp>

namespace templ {

template<typename A, typename B>
class RoleInfoTuple : public Tuple<A, B>
{
    typedef Tuple<A,B> BaseClass;

public:
    typedef shared_ptr< RoleInfoTuple<A,B> > Ptr;

    RoleInfoTuple(const typename BaseClass::APtr& a, const typename BaseClass::BPtr& b)
        : BaseClass(a,b)
    {}

    void addRole(const Role& role) { mRoles.insert(role); }

    bool hasRole(const Role& role) const { return mRoles.end() != mRoles.find(role); }

    const std::set<Role>& getRoles() const { return mRoles; }
    std::string getClassName() const { return "RoleInfoTuple"; }

    std::string toString() const
    {
        std::stringstream ss;
        ss << BaseClass::first()->toString() + "-" << BaseClass::second()->toString() << std::endl;
        ss << "    roles:" << std::endl;
        std::set<Role>::const_iterator it = mRoles.begin();
        for(; it != mRoles.end(); ++it)
        {
            ss << "        " << it->toString() << std::endl;
        }
        return ss.str();
    }

protected:
    graph_analysis::Vertex* getClone() const { return new RoleInfoTuple(*this); }

private:
    std::set<Role> mRoles;

};

} // end namespace templ
#endif // TEMPL_LOCATION_TIMEPOINT_TUPLE_HPP
