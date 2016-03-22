#include "ConstraintNetwork.hpp"

using namespace graph_analysis;

namespace templ {
namespace solvers {

ConstraintNetwork::ConstraintNetwork(graph_analysis::BaseGraph::ImplementationType type)
    : mGraph(BaseGraph::getInstance(type))
{}

ConstraintNetwork::ConstraintNetwork(const ConstraintNetwork& other)
    : mGraph(other.mGraph->cloneEdges())
{
}

ConstraintNetwork::~ConstraintNetwork() {}

ConstraintNetwork::Ptr ConstraintNetwork::clone() const
{
    return ConstraintNetwork::Ptr( getClone() );
}

void ConstraintNetwork::addVariable(const Variable::Ptr& variable)
{
    mGraph->addVertex(variable);
}

void ConstraintNetwork::addConstraint(const Constraint::Ptr& constraint)
{
    mGraph->addEdge(constraint);
}

void ConstraintNetwork::removeConstraint(const Constraint::Ptr& constraint)
{
    mGraph->removeEdge(constraint);
}

graph_analysis::VertexIterator::Ptr ConstraintNetwork::getVariableIterator() const
{
    return mGraph->getVertexIterator();
}

graph_analysis::EdgeIterator::Ptr ConstraintNetwork::getConstraintIterator() const
{
    return mGraph->getEdgeIterator();
}

} // end namespace solvers
} // end namespace templ
