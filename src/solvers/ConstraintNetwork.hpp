#ifndef TEMPL_SOLVERS_CONSTRAINT_NETWORK_HPP
#define TEMPL_SOLVERS_CONSTRAINT_NETWORK_HPP

#include <graph_analysis/lemon/Graph.hpp>
#include <templ/solvers/Constraint.hpp>
#include <templ/solvers/Variable.hpp>

namespace templ {
namespace solvers {

class ConstraintNetwork
{
protected:
    graph_analysis::lemon::DirectedGraph mDigraph;

public:
    virtual ~ConstraintNetwork() {}

    virtual void addVariable(Variable::Ptr variable);

    virtual void addConstraint(Constraint::Ptr constraint);

    virtual graph_analysis::EdgeIterator::Ptr getConstraintIterator() { return mDigraph.getEdgeIterator(); }

};


} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CONSTRAINT_NETWORK_HPP
