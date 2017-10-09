#include "Interval.hpp"
#include <numeric/Combinatorics.hpp>
#include <base-logging/Logging.hpp>

namespace templ {
namespace solvers {
namespace temporal {

Interval::Interval()
    : mpFrom()
    , mpTo()
{}

Interval::Interval(const point_algebra::TimePoint::Ptr& from,
        const point_algebra::TimePoint::Ptr& to,
        const point_algebra::TimePointComparator& comparator)
    : mpFrom(from)
    , mpTo(to)
    , mTimePointComparator(comparator)
{}

void Interval::validate() const
{
    if(!mpFrom || !mpTo)
    {
        throw std::invalid_argument("templ::solvers::temporal::Interval::validate: interval is not fully initialized");
    }
}

bool Interval::contains(const point_algebra::TimePoint::Ptr& t) const
{
    return !mTimePointComparator.greaterThan(t, mpTo) && !mTimePointComparator.lessThan(t,mpFrom);
}

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

std::string Interval::toString(uint32_t indent, bool compact) const
{
    std::stringstream ss;
    std::string hspace(indent, ' ');
    if(!compact)
    {
        ss << hspace << "Interval: " << std::endl;
        if(mpFrom)
        {
            ss << hspace << "    from: " << mpFrom->toString() << std::endl;
        } else {
            ss << hspace << "    from: n/a" << std::endl;
        }
        if(mpTo)
        {
            ss << hspace << "    to: " << mpTo->toString() << std::endl;
        } else {
            ss << hspace << "    to: n/a" << std::endl;
        }
    } else {
        if(mpFrom && mpTo)
        {
            ss << hspace << mpFrom->getLabel() << "-" << mpTo->getLabel();
        } else {
            ss << hspace << "from: n/a - to: n/a";
        }
    }
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
