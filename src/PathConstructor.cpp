#include "PathConstructor.hpp"
#include <templ/LocationTimepointTuple.hpp>

namespace templ {

PathConstructor::PathConstructor(const Role& role)
    : graph_analysis::algorithms::DFSVisitor()
    , mRole(role)
{}

PathConstructor::~PathConstructor() {}

bool PathConstructor::invalidTransition(graph_analysis::Edge::Ptr edge)
{
    LocationTimepointTuple::Ptr targetTuple = dynamic_pointer_cast<LocationTimepointTuple>(edge->getTargetVertex());

    return !targetTuple->hasRole(mRole);
}


void PathConstructor::discoverVertex(graph_analysis::Vertex::Ptr& vertex)
{
    LOG_WARN_S << "Add vertex to path: " << vertex->toString();
    mPath.push_back(vertex);
}

} // end namespace templ
