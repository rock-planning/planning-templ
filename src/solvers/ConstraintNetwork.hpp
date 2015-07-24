#ifndef TEMPL_SOLVERS_CONSTRAINT_NETWORK_HPP
#define TEMPL_SOLVERS_CONSTRAINT_NETWORK_HPP

#include <graph_analysis/lemon/Graph.hpp>
#include <templ/solvers/Constraint.hpp>
#include <templ/solvers/Variable.hpp>

namespace templ {
namespace solvers {

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
    ConstraintNetwork();

    virtual ~ConstraintNetwork() {}

    /**
     * Add a variable to the constraint network
     */
    virtual void addVariable(Variable::Ptr variable);

    /**
     * Add a constraint to the network
     * \throws if constraints does not relate to two
     * exisiting Variables in this network
     */
    virtual void addConstraint(Constraint::Ptr constraint);

    /**
     * Remove a constraint
     */
    virtual void removeConstraint(Constraint::Ptr constraint);

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
};


} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CONSTRAINT_NETWORK_HPP
