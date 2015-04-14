#include "TimePointComparator.hpp"

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

bool TimePointComparator::equals(TimePoint::Ptr t0, TimePoint::Ptr t1) const
{
    return false;
}

bool TimePointComparator::greaterThan(TimePoint::Ptr t0, TimePoint::Ptr t1) const
{
    return false;
}

bool TimePointComparator::hasIntervalOverlap(TimePoint::Ptr a_start, TimePoint::Ptr a_end, TimePoint::Ptr b_start, TimePoint::Ptr b_end) const
{

    return true;
}

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
