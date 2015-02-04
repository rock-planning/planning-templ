#include "ConstraintNetwork.hpp"

namespace terep {
namespace solvers {

void ConstraintNetwork::addVariable(Variable::Ptr variable)
{
    mDigraph.addVertex(variable);
}

void ConstraintNetwork::addConstraint(Constraint::Ptr constraint)
{
    mDigraph.addEdge(constraint);
}

} // end namespace solvers
} // end namespace terep
