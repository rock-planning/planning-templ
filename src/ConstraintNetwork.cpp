#include "ConstraintNetwork.hpp"
#include <graph_analysis/DirectedHyperEdge.hpp>

using namespace graph_analysis;

namespace templ {

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
    Edge::Ptr edge = dynamic_pointer_cast<Edge>(constraint);
    if(edge)
    {
        mGraph->addEdge(edge);
        return;
    }

    DirectedHyperEdge::Ptr hyperEdge = dynamic_pointer_cast<DirectedHyperEdge>(constraint);
    if(hyperEdge)
    {
        mGraph->addHyperEdge(hyperEdge);
        return;
    }
}

void ConstraintNetwork::removeConstraint(const Constraint::Ptr& constraint)
{
    Edge::Ptr edge = dynamic_pointer_cast<Edge>(constraint);
    if(edge)
    {
        mGraph->removeEdge(edge);
        return;
    }

    DirectedHyperEdge::Ptr hyperEdge = dynamic_pointer_cast<DirectedHyperEdge>(constraint);
    if(hyperEdge)
    {
        mGraph->removeHyperEdge(hyperEdge);
        return;
    }
}

graph_analysis::VertexIterator::Ptr ConstraintNetwork::getVariableIterator() const
{
    return mGraph->getVertexIterator();
}

graph_analysis::EdgeIterator::Ptr ConstraintNetwork::getConstraintIterator() const
{
    return mGraph->getEdgeIterator();
}

} // end namespace templ
