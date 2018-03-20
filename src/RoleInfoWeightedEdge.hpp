#ifndef TEMPL_ROLE_INFO_WEIGHTED_EDGE_HPP
#define TEMPL_ROLE_INFO_WEIGHTED_EDGE_HPP

#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/EdgeRegistration.hpp>
#include "RoleInfo.hpp"

namespace templ {

/**
 * Edge representation to allow storing RoleInfo on an edge
 */
class RoleInfoWeightedEdge : public graph_analysis::WeightedEdge, public RoleInfo
{
public:
    typedef shared_ptr<RoleInfoWeightedEdge> Ptr;
    typedef std::vector<Ptr> PtrList;

    RoleInfoWeightedEdge();

    RoleInfoWeightedEdge(const graph_analysis::Vertex::Ptr& source, const graph_analysis::Vertex::Ptr& target, double weight);

    std::string getClassName() const { return "RoleInfoWeightedEdge"; }

    std::string toString() const;

    std::string serializeRoles();
    void deserializeRoles(const std::string& s);

    // Serialization
    //std::string serializeRoles();
    std::string serializeTaggedRoles();

    void deserializeTaggedRoles(const std::string& s);

    virtual void registerAttributes(graph_analysis::EdgeTypeManager* eManager) const;

protected:
    graph_analysis::Edge* getClone() const override { return new RoleInfoWeightedEdge(*this); }

    static const graph_analysis::EdgeRegistration<RoleInfoWeightedEdge> msRoleInfoWeightedEdgeRegistration;
};

} // end namespace
#endif // TEMPL_ROLE_INFO_WEIGHTED_EDGE_HPP

