#ifndef TEMPL_PLANNING_SOLVERS_TEMPORAL_INTERVAL_HPP
#define TEMPL_PLANNING_SOLVERS_TEMPORAL_INTERVAL_HPP

#include "point_algebra/TimePoint.hpp"
#include "point_algebra/TimePointComparator.hpp"
#include <set>
#include <vector>

#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>

namespace templ {
namespace solvers {
namespace temporal {

/**
 * \class Interval
 * \brief An interval between two TimePoints
 * \details
 */
class Interval
{
    point_algebra::TimePoint::Ptr mpFrom;
    point_algebra::TimePoint::Ptr mpTo;

    point_algebra::TimePointComparator mTimePointComparator;

public:
    typedef std::vector<Interval> List;
    typedef std::set<Interval> Set;

    Interval();

    Interval(const point_algebra::TimePoint::Ptr& from,
            const point_algebra::TimePoint::Ptr& to,
            const point_algebra::TimePointComparator& comparator);

    /**
     * Check the valid initialization of this interval
     * \throw std::invalid_argument Throw if interval is not properly initialized
     */
    void validate() const;

    void setFrom(const point_algebra::TimePoint::Ptr& from) { mpFrom = from; }
    const point_algebra::TimePoint::Ptr& getFrom() const { return mpFrom; }

    void setTo(const point_algebra::TimePoint::Ptr& to) { mpTo = to; }
    const point_algebra::TimePoint::Ptr& getTo() const { return mpTo; }


    void setTimePointComparator(const point_algebra::TimePointComparator& comparator) { mTimePointComparator = comparator; }
    const point_algebra::TimePointComparator& getTimePointComparator() const { return mTimePointComparator; }

    bool operator==(const Interval& other) const { return equals(other); }
    /**
     * Implement less operator for general use in set etc.
     */
    bool operator<(const Interval& other) const;

    /**
     * Check if interval [s,t) contains a particular timepoint t, i.e
     * t in [s,t]
     */
    bool contains(const point_algebra::TimePoint::Ptr& t) const;

    /**
     * Check if interval [s,t) contains a particular timepoint t, i.e
     * t in [s,t]
     * \param t TimePoint
     * \param tpc TimePointComparator which is associated with a particular
     * temporal constraint network
     */
    bool contains(const point_algebra::TimePoint::Ptr& t,
                const temporal::point_algebra::TimePointComparator& tpc) const;

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
     * Check before:
     * Symbol: b , inverse: bi
     \verbatim
     [ ---- this ---- ]
                             [ ---- other ---- ]
     \endverbatim
     * \param t TimePoint
     * \param tpc TimePointComparator which is associated with a
     temporal constraint network
     */
    bool before(const Interval& other,
                const temporal::point_algebra::TimePointComparator& tpc) const;

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
     * Check equals
     * Symbol: =, inverse: =
     \verbatim
     [ ---- this ---- ]
     [ --- other ---- ]
     \endverbatim
     */
    bool equals(const Interval& other,
                const temporal::point_algebra::TimePointComparator& tpc) const;

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
     * Check meets
     * Symbol: m, inverse: m
     \verbatim
     [ ---- this ---- ]
                      [ --- other ---- ]
     \endverbatim
     */
    bool meets(const Interval& other,
            const temporal::point_algebra::TimePointComparator& tpc) const;

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
     * Check overlaps
     * Symbol: o, inverse: oi
     \verbatim
     [ ---- this ---- ]
                  [ --- other ---- ]
     \endverbatim
     \param other Other interval
     \param tpc TimePointComparator which is related to a temporal constraint network
     */
    bool overlaps(const Interval& other,
            const temporal::point_algebra::TimePointComparator& tpc) const;

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
     * Check during
     * Symbol: d, inverse: di
     \verbatim
     [ ----           this          ---- ]
                  [ --- other ---- ]
     \endverbatim
     */
    bool during(const Interval& other,
                const temporal::point_algebra::TimePointComparator& tpc) const;

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
     * Check starts
     * Symbol: s, inverse: si
     \verbatim
     [ ----           this          ---- ]
     [ --- other ---- ]
     \endverbatim
     */
    bool starts(const Interval& other,
                const temporal::point_algebra::TimePointComparator& tpc) const;

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
     * Check finishes
     * Symbol: f, inverse: fi
     \verbatim
     [ ----           this          ---- ]
                        [ --- other ---- ]
     \endverbatim
     */
    bool finishes(const Interval& other,
                const temporal::point_algebra::TimePointComparator& tpc) const;

    /**
     * Negation of equals
     */
    bool distinctFrom(const Interval& other) const;

    /**
     * Negation of equals
     */
    bool distinctFrom(const Interval& other,
                const temporal::point_algebra::TimePointComparator& tpc) const;

    /**
     * Stringify interval object
     */
    std::string toString(uint32_t indent = 0, bool compact = false) const;

    /**
     * Stringify list
     */
    static std::string toString(const List& list, size_t indent = 0, bool compact = false);

    /**
     * Check if all given intervals overlap with each other
     * \return true if all intervals in the given list overlap with each other
     *      false otherwise
     */
    static bool areOverlapping(const std::vector<uint32_t>& indices, const std::vector<Interval>& intervals);

    /**
     * Get all combinations of intervals that overlap with each other
     * \return set of overlapping interval combinations using the index of given
     * set of intervals
     */
    static std::set< std::vector<uint32_t> > overlappingIntervals(const std::vector<Interval>& intervals);

};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ

namespace std
{
/// Define hash function so that class can be used in std::ordered_set
/// \see http://en.cppreference.com/w/cpp/utility/hash
template<>
struct hash<templ::solvers::temporal::Interval>
{
    size_t operator()(const templ::solvers::temporal::Interval &i) const
    {
        using namespace templ::solvers::temporal::point_algebra;

        // std::hash<T*> is defined as well as for std::shared_ptr
        size_t h1 = std::hash<TimePoint*>()(i.getFrom().get());
        size_t h2 = std::hash<TimePoint*>()(i.getTo().get());
        return h1 ^ ( h2 << 1 );
    }
};

}
#endif // TEMPL_SOLVERS_TEMPORAL_INTERVAL_HPP
