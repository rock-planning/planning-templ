#ifndef TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_EXACT_TIME_POINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_EXACT_TIME_POINT_HPP

#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

class ExactTimePoint : public TimePoint
{
public:
    ExactTimePoint(uint64_t exactTimePoint)
        : TimePoint(exactTimePoint, exactTimePoint)
    {}
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_EXACT_TIME_POINT_HPP
