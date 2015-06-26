#ifndef TEMPL_PLANNING_SOLVERS_TEMPORAL_INTERVAL_HPP
#define TEMPL_PLANNING_SOLVERS_TEMPORAL_INTERVAL_HPP

#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>

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

    point_algebra::TimePointComparator mTimePointComparator;

public:
    Interval(const point_algebra::TimePoint::Ptr& from,
            const point_algebra::TimePoint::Ptr& to,
            const point_algebra::TimePointComparator& comparator);

    point_algebra::TimePoint::Ptr getFrom() const { return mpFrom; }
    point_algebra::TimePoint::Ptr getTo() const { return mpTo; }

    bool operator==(const Interval& other) const { return equals(other); }

    /**
     * Check before:
     * Symbol: b , inverse: bi
     \verbatim
     [ ---- this ---- ]
                             [ ---- other ---- ]
     \endverbatim
     */
    bool before(const Interval& other) const;

    /**
     * Check equals
     * Symbol: =, inverse: =
     \verbatim
     [ ---- this ---- ]
     [ --- other ---- ]
     \endverbatim
     */
    bool equals(const Interval& other) const;

    /**
     * Check meets
     * Symbol: m, inverse: m
     \verbatim
     [ ---- this ---- ]
                      [ --- other ---- ]
     \endverbatim
     */
    bool meets(const Interval& other) const;

    /**
     * Check overlaps
     * Symbol: o, inverse: oi
     \verbatim
     [ ---- this ---- ]
                  [ --- other ---- ]
     \endverbatim
     */
    bool overlaps(const Interval& other) const;

    /**
     * Check during
     * Symbol: d, inverse: di
     \verbatim
     [ ----           this          ---- ]
                  [ --- other ---- ]
     \endverbatim
     */
    bool during(const Interval& other) const;

    /**
     * Check starts
     * Symbol: s, inverse: si
     \verbatim
     [ ----           this          ---- ]
     [ --- other ---- ]
     \endverbatim
     */
    bool starts(const Interval& other) const;

    /**
     * Check finishes
     * Symbol: f, inverse: fi
     \verbatim
     [ ----           this          ---- ]
                        [ --- other ---- ]
     \endverbatim
     */
    bool finishes(const Interval& other) const;

    /**
     * Negation of equals
     */
    bool distinctFrom(const Interval& other) const;

    /**
     * Stringify interval object
     */
    std::string toString() const;
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_INTERVAL_HPP
