#include "Interval.hpp"
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

} // end namespace temporal
} // end namespace solvers
} // end namespace templ