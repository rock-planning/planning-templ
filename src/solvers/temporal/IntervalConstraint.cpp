#include "IntervalConstraint.hpp"

using namespace graph_analysis;

namespace templ {
namespace solvers {


IntervalConstraint::IntervalConstraint(Variable::Ptr source, Variable::Ptr target, double newLowerBound, double newUpperBound)
    : Constraint(source, target)
{
    lowerBound = newLowerBound;
    upperBound = newUpperBound;
}

std::string IntervalConstraint::toString() const 
{ 
	std::stringstream ss;
	ss << "Interval Constraint: from " << getSourceVertex()->toString() << " to " << getTargetVertex()->toString() << " ";
	ss << "with interval: [" << lowerBound << "," << upperBound << "]";
    return ss.str();
}

} // end namespace solvers
} // end namespace templ