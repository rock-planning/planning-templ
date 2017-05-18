#ifndef TEMPL_ROLE_INFO_TUPLE_HPP
#define TEMPL_ROLE_INFO_TUPLE_HPP

#include <set>
#include <graph_analysis/WeightedEdge.hpp>
#include "Role.hpp"
#include "Tuple.hpp"
#include "RoleInfo.hpp"

namespace templ {

template<typename A, typename B>
class RoleInfoTuple : public Tuple<A, B>, public RoleInfo
{
    typedef Tuple<A,B> BaseClass;

public:
    typedef shared_ptr< RoleInfoTuple<A,B> > Ptr;

    RoleInfoTuple(const typename BaseClass::a_t& a, const typename BaseClass::b_t& b)
        : BaseClass(a,b)
        , RoleInfo()
    {}

    std::string getClassName() const override { return "RoleInfoTuple"; }

    std::string toString() const override
    {
        std::stringstream ss;
        ss << BaseClass::toString() << std::endl;
        ss << RoleInfo::toString();

        return ss.str();
    }

protected:
    graph_analysis::Vertex* getClone() const override { return new RoleInfoTuple(*this); }

};

} // end namespace templ
#endif // TEMPL_ROLE_INFO_TUPLE_HPP
