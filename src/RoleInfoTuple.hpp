#ifndef TEMPL_ROLE_INFO_TUPLE_HPP
#define TEMPL_ROLE_INFO_TUPLE_HPP

#include <set>
#include <graph_analysis/Vertex.hpp>
#include <graph_analysis/VertexRegistration.hpp>

#include "Role.hpp"
#include "Tuple.hpp"
#include "RoleInfo.hpp"
#include <base-logging/Logging.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <iomanip>

namespace templ {

/**
 * This class combines the concept of Tuple and RoleInfo,
 * so that both can stored on a single vertex in a graph
 */
template<typename A, typename B>
class RoleInfoTuple : public Tuple<A, B>, public virtual RoleInfo
{
    typedef Tuple<A,B> BaseClass;

public:
    typedef shared_ptr< RoleInfoTuple<A,B> > Ptr;

    static const graph_analysis::VertexRegistration< RoleInfoTuple<A,B> > msRoleInfoTupleRegistration;

    RoleInfoTuple()
        : RoleInfo()
        , BaseClass()
    {}

    RoleInfoTuple(const typename BaseClass::a_t& a, const typename BaseClass::b_t& b)
        : RoleInfo()
        , BaseClass(a,b)
    {
    }

    std::string getClassName() const override { return "RoleInfoTuple"; }

    std::string toString(uint32_t indent = 0) const override
    {
        std::string hspace(indent,' ');
        std::stringstream ss;
        ss << BaseClass::toString(indent) << std::endl;
        ss << RoleInfo::toString(indent);

        return ss.str();
    }

    std::string serializeTuple()
    {
        std::stringstream ss;
        boost::archive::text_oarchive oarch(ss);
        oarch << *tuple_get_pointer( BaseClass::first() );
        oarch << *tuple_get_pointer( BaseClass::second() );
        return ss.str();
    }

    void deserializeTuple(const std::string& s)
    {
        std::stringstream ss;
        ss << s;
        boost::archive::text_iarchive iarch(ss);

        typename A::element_type a;
        typename B::element_type b;

        iarch >> a;
        iarch >> b;

        // We assume we use Location and TimePoint here
        BaseClass::setFirst( A::element_type::create(a) );
        BaseClass::setSecond( B::element_type::create(b) );
    }

    std::string serializeRoles()
    {
        std::stringstream ss;
        boost::archive::text_oarchive oarch(ss);
        oarch << mRoles;
        return ss.str();
    }

    void deserializeRoles(const std::string& s)
    {
        std::stringstream ss;
        ss << s;
        boost::archive::text_iarchive iarch(ss);
        iarch >> mRoles;
    }

    // Serialization
    //std::string serializeRoles();
    std::string serializeTaggedRoles()
    {
        std::stringstream ss;
        boost::archive::text_oarchive oarch(ss);
        oarch << mTaggedRoles;
        return ss.str();
    }

    void deserializeTaggedRoles(const std::string& s)
    {
        std::stringstream ss;
        ss << s;
        boost::archive::text_iarchive iarch(ss);
        iarch >> mTaggedRoles;
    }

    // Todo: for registration to work the particular VertexRegistration for this
    // type must be added to the source cpp file
    virtual void registerAttributes(graph_analysis::VertexTypeManager* vManager) const override
    {
        vManager->registerAttribute(getClassName(), "roles",
                   (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoTuple::serializeRoles,
                   (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoTuple::deserializeRoles,
                   (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&RoleInfoTuple::serializeRoles);
        vManager->registerAttribute(getClassName(), "tagged_roles",
                   (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoTuple::serializeTaggedRoles,
                   (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoTuple::deserializeTaggedRoles,
                   (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&RoleInfoTuple::serializeTaggedRoles);

        vManager->registerAttribute(getClassName(), "tuple",
                (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)
                &RoleInfoTuple::serializeTuple,
                (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)
                &RoleInfoTuple::deserializeTuple,
                (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)
                &RoleInfoTuple::serializeTuple);
    }


protected:
    graph_analysis::Vertex* getClone() const override { return new RoleInfoTuple(*this); }
};

} // end namespace templ
#endif // TEMPL_ROLE_INFO_TUPLE_HPP
