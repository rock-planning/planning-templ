#include "Constraint.hpp"

using namespace graph_analysis;

namespace templ {
namespace solvers {

Constraint::Constraint(Variable::Ptr source, Variable::Ptr target)
    : Edge(source, target)
{}


std::string Constraint::toString() const 
{ 
    return "Constraint: from " + getSourceVertex()->toString() + " to " + getTargetVertex()->toString();
}

} // end namespace solvers
} // end namespace templ
