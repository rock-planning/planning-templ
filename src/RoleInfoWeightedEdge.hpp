#ifndef TEMPL_ROLE_INFO_WEIGHTED_EDGE_HPP
#define TEMPL_ROLE_INFO_WEIGHTED_EDGE_HPP

#include <graph_analysis/WeightedEdge.hpp>
#include "RoleInfo.hpp"

namespace templ {

class RoleInfoWeightedEdge : public graph_analysis::WeightedEdge, public RoleInfo
{
public:
    typedef shared_ptr<RoleInfoWeightedEdge> Ptr;

    RoleInfoWeightedEdge()
        : graph_analysis::WeightedEdge()
        , RoleInfo()
    {}

    RoleInfoWeightedEdge(const graph_analysis::Vertex::Ptr& source, const graph_analysis::Vertex::Ptr& target, double weight)
        : graph_analysis::WeightedEdge(source, target, weight)
        , RoleInfo()
    {}

    std::string getClassName() const { return "RoleInfoWeightedEdge"; }

    std::string toString() const
    {
        std::stringstream ss;
        ss << getWeight() << std::endl;
        ss << RoleInfo::toString();

        return ss.str();
    }
protected:
    graph_analysis::Edge* getClone() const override { return new RoleInfoWeightedEdge(*this); }

};

} // end namespace
#endif // TEMPL_ROLE_INFO_WEIGHTED_EDGE_HPP

