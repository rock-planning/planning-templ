#ifndef TEMPL_PLANNING_SOLVERS_TEMPORAL_INTERVAL_HPP
#define TEMPL_PLANNING_SOLVERS_TEMPORAL_INTERVAL_HPP

namespace templ {
namespace solvers {
namespace temporal {

/**
 * Create an interval
 */
class Interval
{
    point_algebra::TimePoint mFrom
    point_algebra::TimePoint mTo;

public:
    Interval(const point_algebra::TimePoint& from, const point_algebra::TimePoint& to)
        : mFrom(from)
        , mTo(to)
    {}

    point_algebra::TimePoint getFrom() const { return mFrom; }
    point_algebra::TimePoint getTo() const { return mTo; }

};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_INTERVAL_HPP
