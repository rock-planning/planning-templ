#include "IntervalConstraint.hpp"

using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace temporal {


IntervalConstraint::IntervalConstraint(point_algebra::TimePoint::Ptr source, point_algebra::TimePoint::Ptr target, double newLowerBound, double newUpperBound)
    : Edge(source, target)
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

} // end namespace temporal
} // end namespace solvers
} // end namespace templ