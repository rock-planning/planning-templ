#ifndef TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_QUALITATIVE_TIME_POINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_QUALITATIVE_TIME_POINT_HPP

#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

class QualitativeTimePoint : public TimePoint
{
public:
    QualitativeTimePoint()
        : TimePoint(0,0)
    {}
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ

#endif // TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_QUALITATIVE_TIME_POINT_HPP
