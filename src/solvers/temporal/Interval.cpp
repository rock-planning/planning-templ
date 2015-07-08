#include "Interval.hpp"
#include <numeric/Combinatorics.hpp>
#include <base/Logging.hpp>

namespace templ {
namespace solvers {
namespace temporal {

Interval::Interval(const point_algebra::TimePoint::Ptr& from,
        const point_algebra::TimePoint::Ptr& to,
        const point_algebra::TimePointComparator& comparator)
    : mpFrom(from)
    , mpTo(to)
    , mTimePointComparator(comparator)
{}

bool Interval::before(const Interval& other) const
{
    // assuming consistency of the interval, i.e.
    // mpTo before mpFrom
    return mTimePointComparator.lessThan(mpTo, other.mpFrom);
}

bool Interval::equals(const Interval& other) const
{
    return mTimePointComparator.equals(mpTo, other.mpTo) &&
        mTimePointComparator.equals(mpFrom, other.mpFrom);
}

bool Interval::meets(const Interval& other) const
{
    return mTimePointComparator.equals(mpTo, other.mpFrom);
}

bool Interval::overlaps(const Interval& other) const
{
    return mTimePointComparator.hasIntervalOverlap(mpFrom, mpTo,
            other.mpFrom, other.mpTo);
}

bool Interval::during(const Interval& other) const
{
    return mTimePointComparator.greaterThan(mpFrom, other.mpFrom)
        && mTimePointComparator.lessThan(mpTo, other.mpTo);
}

bool Interval::starts(const Interval& other) const
{
    return mTimePointComparator.equals(mpFrom, other.mpFrom);
}

bool Interval::finishes(const Interval& other) const
{
    return mTimePointComparator.equals(mpTo, other.mpTo);
}

bool Interval::distinctFrom(const Interval& other) const
{
    return !equals(other);
}

std::string Interval::toString() const
{
    std::stringstream ss;
    ss << "Interval: " << std::endl;
    ss << "    from: " << mpFrom->toString() << std::endl;
    ss << "    to: " << mpTo->toString() << std::endl;
    return ss.str();
}

bool Interval::areOverlapping(const std::vector<uint32_t>& indices, const std::vector<Interval>& allIntervals)
{
    if(indices.size() < 2)
    {
        return false;
    }

    numeric::Combination<uint32_t> combinations(indices, 2, numeric::EXACT);
    do {
        std::vector<uint32_t> combination = combinations.current();
        if(!allIntervals[ combination[0] ].overlaps(allIntervals[ combination[1] ] ))
        {
            return false;
        }
    } while(combinations.next());
    return true;
}

std::set< std::vector<uint32_t> > Interval::overlappingIntervals(const std::vector<Interval>& intervals)
{
    std::set< std::vector<uint32_t> > overlappingIntervals;
    if(intervals.size() < 2)
    {
        return overlappingIntervals;
    }

    std::vector<uint32_t> indices;
    for(size_t i = 0; i < intervals.size(); ++i)
    {
        indices.push_back(i);
    }

    numeric::Combination<uint32_t> combinations(indices, indices.size(), numeric::MAX);
    do {
        std::vector<uint32_t> combination = combinations.current();
        if( areOverlapping(combination, intervals) )
        {
            overlappingIntervals.insert(combination);
        }
    } while(combinations.next());
    return overlappingIntervals;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
