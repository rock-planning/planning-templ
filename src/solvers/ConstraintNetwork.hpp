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
    graph_analysis::BaseGraph::Ptr mGraph;

    graph_analysis::BaseGraph::Ptr getGraph() const { return mGraph; }

public:
    ConstraintNetwork();

    virtual ~ConstraintNetwork() {}

    virtual void addVariable(Variable::Ptr variable);

    virtual void addConstraint(Constraint::Ptr constraint);

    virtual graph_analysis::EdgeIterator::Ptr getConstraintIterator() const;

};


} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CONSTRAINT_NETWORK_HPP
