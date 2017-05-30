#ifndef TEMPL_ROLE_INFO_TUPLE_HPP
#define TEMPL_ROLE_INFO_TUPLE_HPP

#include <set>
#include <graph_analysis/Vertex.hpp>
#include <graph_analysis/VertexTypeManager.hpp>
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

template<typename A, typename B>
class RoleInfoTuple : public Tuple<A, B>, public virtual RoleInfo
{
    typedef Tuple<A,B> BaseClass;

public:
    typedef shared_ptr< RoleInfoTuple<A,B> > Ptr;

    RoleInfoTuple()
        : RoleInfo()
        , BaseClass()
    {
        init();
    }

    RoleInfoTuple(const typename BaseClass::a_t& a, const typename BaseClass::b_t& b)
        : RoleInfo()
        , BaseClass(a,b)
    {
        init();
    }

    std::string getClassName() const override { return "RoleInfoTuple"; }

    std::string toString() const override
    {
        std::stringstream ss;
        ss << BaseClass::toString() << std::endl;
        ss << RoleInfo::toString();

        return ss.str();
    }

    void init()
    {
        static VertexRegistration edgeRegistration;
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

protected:
    graph_analysis::Vertex* getClone() const override { return new RoleInfoTuple(*this); }

    RoleInfoTuple(bool doInit)
        : RoleInfo()
        , BaseClass()
    {
        if(doInit)
        {
            init();
        }
    }


    class VertexRegistration
    {
        bool mRegistered;

    public:
        VertexRegistration()
            : mRegistered(false)
        {
            if(!mRegistered)
            {
                mRegistered = true;

                using namespace graph_analysis;

                LOG_WARN_S << "Performing vertex registration";
                VertexTypeManager* vManager = VertexTypeManager::getInstance();
                RoleInfoTuple::Ptr vertex( new RoleInfoTuple(false));
                vManager->registerType("RoleInfoTuple", vertex, true);
                vManager->registerAttribute("RoleInfoTuple", "roles",
                           (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoTuple::serializeRoles,
                           (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoTuple::deserializeRoles,
                           (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&RoleInfoTuple::serializeRoles);
                vManager->registerAttribute("RoleInfoTuple", "tagged_roles",
                           (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoTuple::serializeTaggedRoles,
                           (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoTuple::deserializeTaggedRoles,
                           (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&RoleInfoTuple::serializeTaggedRoles);

                vManager->registerAttribute("RoleInfoTuple", "tuple",
                        (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)
                        &RoleInfoTuple::serializeTuple,
                        (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)
                        &RoleInfoTuple::deserializeTuple,
                        (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)
                        &RoleInfoTuple::serializeTuple);

            }
        }
    };

};

} // end namespace templ
#endif // TEMPL_ROLE_INFO_TUPLE_HPP
