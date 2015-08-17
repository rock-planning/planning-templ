#include "IntervalConstraint.hpp"

using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace temporal {


IntervalConstraint::IntervalConstraint(point_algebra::TimePoint::Ptr source, point_algebra::TimePoint::Ptr target)
    : Edge(source, target)
{
}

std::string IntervalConstraint::toString() const 
{ 
	std::stringstream ss;
	ss << "Interval Constraint: from " << getSourceVertex()->toString() << " to " << getTargetVertex()->toString() << " ";
    return ss.str();
}


bool IntervalConstraint::checkInterval(Bounds x)
{
	std::vector<Bounds>::iterator it = intervals.begin();
	while (it!=intervals.end())
	{
		if (it->getLowerBound() == x.getLowerBound() && it->getUpperBound() == x.getUpperBound()) return true;
		it++;
	}
	return false;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ