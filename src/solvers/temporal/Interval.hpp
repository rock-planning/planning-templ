#ifndef TEMPL_PLANNING_SOLVERS_TEMPORAL_INTERVAL_HPP
#define TEMPL_PLANNING_SOLVERS_TEMPORAL_INTERVAL_HPP

#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>

namespace templ {
namespace solvers {
namespace temporal {

/**
 * Create an interval
 */
class Interval
{
    point_algebra::TimePoint::Ptr mpFrom;
    point_algebra::TimePoint::Ptr mpTo;

public:
    Interval(const point_algebra::TimePoint::Ptr& from,const point_algebra::TimePoint::Ptr& to)
        : mpFrom(from)
        , mpTo(to)
    {}

    point_algebra::TimePoint::Ptr getFrom() const { return mpFrom; }
    point_algebra::TimePoint::Ptr getTo() const { return mpTo; }

    bool operator<(const Interval& other) const
    {
        if(*mpFrom.get() < *other.mpFrom.get())
        {
            return true;
        } else {
            return *mpTo.get() < *other.mpTo.get();
        }
    }

};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_INTERVAL_HPP
