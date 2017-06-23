#include "PathConstructor.hpp"
#include <templ/RoleInfoTuple.hpp>

namespace templ {

PathConstructor::PathConstructor(const Role& role, const std::string& roleSetLabel)
    : graph_analysis::algorithms::DFSVisitor()
    , mRole(role)
    , mRoleSetLabel(roleSetLabel)
{}

PathConstructor::~PathConstructor() {}

bool PathConstructor::isInvalidTransition(const graph_analysis::Edge::Ptr& edge)
{
    RoleInfo::Ptr targetTuple = dynamic_pointer_cast<RoleInfo>(edge->getTargetVertex());

    return targetTuple && !targetTuple->hasRole(mRole, mRoleSetLabel);
}


void PathConstructor::discoverVertex(graph_analysis::Vertex::Ptr& vertex)
{
    LOG_DEBUG_S << "Add vertex to path: " << vertex->toString();
    mPath.push_back(vertex);
}

} // end namespace templ
