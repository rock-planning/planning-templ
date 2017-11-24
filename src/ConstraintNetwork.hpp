#ifndef TEMPL_CONSTRAINT_NETWORK_HPP
#define TEMPL_CONSTRAINT_NETWORK_HPP

#include <graph_analysis/BaseGraph.hpp>
#include "Constraint.hpp"
#include "Variable.hpp"

namespace templ {

/**
 * The constraint network represent a generic network of constraints, i.e.
 * Variables as nodes/vertices and Constraints as links/edges
 *
 */
class ConstraintNetwork
{
protected:
    // Internal representation of the constraint network
    graph_analysis::BaseGraph::Ptr mGraph;

public:
    typedef shared_ptr<ConstraintNetwork> Ptr;

    /**
     * Default constructor
     * (setting the type of the underlying base graph)
     */
    ConstraintNetwork(graph_analysis::BaseGraph::ImplementationType type = graph_analysis::BaseGraph::BOOST_DIRECTED_GRAPH);

    virtual ~ConstraintNetwork();

    /**
     * Copy constructor for a ConstraintNetwork
     * (requires to clone the underling base graph)
     */
    ConstraintNetwork(const ConstraintNetwork& network);

    /**
     * Create a clone (deep copy) of this constraint network
     */
    ConstraintNetwork::Ptr clone() const;

    /**
     * Add a variable to the constraint network
     */
    virtual void addVariable(const Variable::Ptr& variable);

    /**
     * Add a constraint to the network
     * \throws if constraints does not relate to two
     * exisiting Variables in this network
     */
    virtual void addConstraint(const Constraint::Ptr& constraint);

    /**
     * Remove a constraint
     */
    virtual void removeConstraint(const Constraint::Ptr& constraint);

    /**
     * Get the variable iterator for this constraint network
     * \return iterator
     */
    virtual graph_analysis::VertexIterator::Ptr getVariableIterator() const;

    /**
     * Get the constraint iterator for this constraint network
     * \return Iterator
     */
    virtual graph_analysis::EdgeIterator::Ptr getConstraintIterator() const;

    /**
     * Get the underlying graph of this constraint network
     */
    graph_analysis::BaseGraph::Ptr getGraph() const { return mGraph; }

protected:
    /**
     * Set the underlying graph of this constraint network
     */
    graph_analysis::BaseGraph::Ptr setGraph(const graph_analysis::BaseGraph::Ptr& graph) { return mGraph = graph; }

    virtual ConstraintNetwork* getClone() const { return new ConstraintNetwork(*this); }

};

} // end namespace templ
#endif // TEMPL_CONSTRAINT_NETWORK_HPP
