#include "PathConstructor.hpp"
#include <templ/RoleInfoTuple.hpp>

namespace templ {

PathConstructor::PathConstructor(const Role& role)
    : graph_analysis::algorithms::DFSVisitor()
    , mRole(role)
{}

PathConstructor::~PathConstructor() {}

bool PathConstructor::isInvalidTransition(const graph_analysis::Edge::Ptr& edge)
{
    RoleInfo::Ptr targetTuple = dynamic_pointer_cast<RoleInfo>(edge->getTargetVertex());

    return targetTuple && !targetTuple->hasRole(mRole);
}


void PathConstructor::discoverVertex(graph_analysis::Vertex::Ptr& vertex)
{
    LOG_WARN_S << "Add vertex to path: " << vertex->toString();
    mPath.push_back(vertex);
}

} // end namespace templ
