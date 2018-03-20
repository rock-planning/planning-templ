#include "RoleInfoWeightedEdge.hpp"
#include <graph_analysis/EdgeTypeManager.hpp>
#include <base-logging/Logging.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <iomanip>

namespace templ {

const graph_analysis::EdgeRegistration<RoleInfoWeightedEdge> RoleInfoWeightedEdge::msRoleInfoWeightedEdgeRegistration;

RoleInfoWeightedEdge::RoleInfoWeightedEdge()
    : graph_analysis::WeightedEdge()
    , RoleInfo()
{
}

RoleInfoWeightedEdge::RoleInfoWeightedEdge(const graph_analysis::Vertex::Ptr& source, const graph_analysis::Vertex::Ptr& target, double weight)
    : graph_analysis::WeightedEdge(source, target, weight)
    , RoleInfo()
{
}

std::string RoleInfoWeightedEdge::toString() const
{
    std::stringstream ss;
    double weight = getWeight();
    if(weight == std::numeric_limits<double>::max())
    {
        // inf
        ss << "capacity: +inf" << std::endl; //"\u221E" << std::endl;
    } else {
        ss << "capacity: " << std::setprecision(2) << weight << std::endl;
    }
    ss << RoleInfo::toString();

    return ss.str();
}

void RoleInfoWeightedEdge::registerAttributes(graph_analysis::EdgeTypeManager* eManager) const
{
    using namespace graph_analysis;

    eManager->registerAttribute("RoleInfoWeightedEdge", "weights",
               (io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoWeightedEdge::serializeWeights,
               (io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoWeightedEdge::deserializeWeights,
               (io::AttributeSerializationCallbacks::print_func_t)&RoleInfoWeightedEdge::serializeWeights);

    eManager->registerAttribute("RoleInfoWeightedEdge", "roles",
               (io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoWeightedEdge::serializeRoles,
               (io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoWeightedEdge::deserializeRoles,
               (io::AttributeSerializationCallbacks::print_func_t)&RoleInfoWeightedEdge::serializeRoles);

    eManager->registerAttribute("RoleInfoWeightedEdge", "tagged_roles",
               (io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoWeightedEdge::serializeTaggedRoles,
               (io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoWeightedEdge::deserializeTaggedRoles,
               (io::AttributeSerializationCallbacks::print_func_t)&RoleInfoWeightedEdge::serializeTaggedRoles);

}

std::string RoleInfoWeightedEdge::serializeRoles()
{
    std::stringstream ss;
    boost::archive::text_oarchive oarch(ss);
    oarch << mRoles;
    return ss.str();
}

std::string RoleInfoWeightedEdge::serializeTaggedRoles()
{
    std::stringstream ss;
    boost::archive::text_oarchive oarch(ss);
    oarch << mTaggedRoles;
    return ss.str();
}

void RoleInfoWeightedEdge::deserializeRoles(const std::string& s)
{
    std::stringstream ss;
    ss << s;
    boost::archive::text_iarchive iarch(ss);
    iarch >> mRoles;
}

void RoleInfoWeightedEdge::deserializeTaggedRoles(const std::string& s)
{
    std::stringstream ss;
    ss << s;
    boost::archive::text_iarchive iarch(ss);
    iarch >> mTaggedRoles;
}

}  // end namespace templ
