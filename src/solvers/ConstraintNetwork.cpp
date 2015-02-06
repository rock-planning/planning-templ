#include "ConstraintNetwork.hpp"

namespace templ {
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
} // end namespace templ
