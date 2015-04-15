#include "ConstraintNetwork.hpp"

namespace templ {
namespace solvers {

ConstraintNetwork::ConstraintNetwork()
    : mGraph(new graph_analysis::lemon::DirectedGraph())
{}

void ConstraintNetwork::addVariable(Variable::Ptr variable)
{
    mGraph->addVertex(variable);
}

void ConstraintNetwork::addConstraint(Constraint::Ptr constraint)
{
    mGraph->addEdge(constraint);
}

graph_analysis::EdgeIterator::Ptr ConstraintNetwork::getConstraintIterator() const
{
    return mGraph->getEdgeIterator();
}

} // end namespace solvers
} // end namespace templ
