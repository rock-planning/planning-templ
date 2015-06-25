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

bool Interval::distinctFrom(const Interval& other) const
{
    return !mTimePointComparator.hasIntervalOverlap(mpFrom, mpTo, other.mpFrom, other.mpTo);
}

bool Interval::lessThan(const Interval& other) const
{
    LOG_DEBUG_S << "Comparing interval " << std::endl
        << toString() << " with " << std::endl
        << other.toString();
    if(!distinctFrom(other))
    {
        LOG_DEBUG_S << "Interval " << toString() << " is not distinct from other: "
            << other.toString();
        return false;
    } else {
        return mTimePointComparator.lessThan(mpTo, other.mpFrom);
    }
}

bool Interval::equals(const Interval& other) const
{
    return mTimePointComparator.equals(mpTo, other.mpTo) &&
        mTimePointComparator.equals(mpFrom, other.mpFrom);
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
