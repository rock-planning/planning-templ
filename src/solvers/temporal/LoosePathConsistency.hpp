#ifndef TEMPL_SOLVERS_TEMPORAL_LOOSE_PATH_CONSISTENCY
#define TEMPL_SOLVERS_TEMPORAL_LOOSE_PATH_CONSISTENCY

#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>

namespace templ {
namespace solvers {
namespace temporal {

class LoosePathConsistency
{
public:
	// compute the intersection between two sets of intervals
	static std::vector<Bounds> intersection(std::vector<Bounds> a, std::vector<Bounds> b);

	// check if an interval x is included in a set of intervals a
	static bool checkInterval(std::vector<Bounds> a, Bounds x);

	// compute the composition between two sets of intervals
	static std::vector<Bounds> composition(std::vector<Bounds> a, std::vector<Bounds> b);

	// compute the loose intersection between two sets of intervals
	static std::vector<Bounds> looseIntersection(std::vector<Bounds> a, std::vector<Bounds> b);

	// change [a,b] into [-b,-a] for each interval of a set v
	static std::vector<Bounds> reverseIntervals(std::vector<Bounds> v);

	// step 1
	static TemporalConstraintNetwork intersectionNetwork(TemporalConstraintNetwork t);

	// step 2
	static TemporalConstraintNetwork looseNetwork(TemporalConstraintNetwork a, TemporalConstraintNetwork b);

	// final function
	static TemporalConstraintNetwork loosePathConsistency(TemporalConstraintNetwork t);

};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_LOOSE_PATH_CONSISTENCY