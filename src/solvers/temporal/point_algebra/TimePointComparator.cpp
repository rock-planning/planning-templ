#include "TimePointComparator.hpp"

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

bool TimePointComparator::equals(TimePoint::Ptr t0, TimePoint::Ptr t1) const
{
    return t0->equals(t1);
}

bool TimePointComparator::greaterThan(TimePoint::Ptr t0, TimePoint::Ptr t1) const
{
    if(t0->getType() != t1->getType())
    {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePointComparator::greaterThan: cannot compare different types of TimePoints");
    }

    if(t0->getType() == QUANTITATIVE)
    {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePointComparator::greaterThan: cannot compare different types of TimePoints");
    } else if(t1->getType() == QUALITATIVE)
    {
        // Check in DB if we have some formulated constraints between two
        // timepoints
        // --> use QualitativeTimePointConstraintNetwork


    } else {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePointComparator::greaterThan: cannot compare this type of TimePoints");
    }
}

bool TimePointComparator::hasIntervalOverlap(TimePoint::Ptr a_start, TimePoint::Ptr a_end, TimePoint::Ptr b_start, TimePoint::Ptr b_end) const
{
    if(TimePointComparator::inInterval(a_start, b_start, b_end))
    {
        return true;
    } else if(TimePointComparator::inInterval(a_end, b_start, b_end))
    {
        return true;
    } else if(TimePointComparator::inInterval(b_start, a_start, a_end))
    {
        return true;
    } else if(TimePointComparator::inInterval(b_end, a_start, a_end))
    {
        return true;
    }

    return false;
}

bool TimePointComparator::inInterval(TimePoint::Ptr t0, TimePoint::Ptr i_start, TimePoint::Ptr i_end) const
{
    if(TimePointComparator::greaterOrEqual(t0, i_start) && TimePointComparator::lessOrEqual(t0, i_end))
    {
        return true;
    }
    return false;
}

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
