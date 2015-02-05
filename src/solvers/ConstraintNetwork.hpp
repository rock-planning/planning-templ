#ifndef TEREP_SOLVERS_CONSTRAINT_NETWORK_HPP
#define TEREP_SOLVERS_CONSTRAINT_NETWORK_HPP

#include <graph_analysis/lemon/Graph.hpp>
#include <terep/solvers/Constraint.hpp>
#include <terep/solvers/Variable.hpp>

namespace terep {
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
} // end namespace terep
#endif // TEREP_SOLVERS_CONSTRAINT_NETWORK_HPP
