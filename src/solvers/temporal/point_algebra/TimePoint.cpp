#include "TimePoint.hpp"
#include "QualitativeTimePoint.hpp"
#include <sstream>
#include <algorithm>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

TimePoint::PtrList TimePoint::msTimePoints;

TimePoint::TimePoint()
    : mLowerBound(0)
    , mUpperBound(0)
    , mType(QUANTITATIVE)
{}

TimePoint::TimePoint(uint64_t lowerBound, uint64_t upperBound, Type type)
    : mLowerBound(lowerBound)
    , mUpperBound(upperBound)
    , mType(type)
{
    TimePoint::validateBounds(mLowerBound, mUpperBound);
}

TimePoint::TimePoint(uint64_t lowerBound, uint64_t upperBound)
    : mLowerBound(lowerBound)
    , mUpperBound(upperBound)
    , mType(QUANTITATIVE)

{
    TimePoint::validateBounds(mLowerBound, mUpperBound);
}

void TimePoint::validateBounds(uint64_t lowerBound, uint64_t upperBound)
{
    if(upperBound < lowerBound)
    {
        std::stringstream ss;
        ss << "TimePoint: upper bound (" << upperBound << ") smaller than lower bound (" << lowerBound << ")";
        throw std::invalid_argument(ss.str());
    }
}

bool TimePoint::equals(const TimePoint::Ptr& other) const
{
    if(mType != other->getType())
    {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePoint::equals: cannot compare timepoints of different types");
    }

    // TODO: improve comparision integration
    if(mType == QUANTITATIVE)
    {
        return *this == *other.get();
    } else if(mType == QUALITATIVE)
    {
        return *dynamic_cast<QualitativeTimePoint*>(const_cast<TimePoint*>(this)) == *dynamic_cast<QualitativeTimePoint*>(other.get());
    } else {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePoint::equals: cannot compare timepoints of unknown types");
    }
}

bool TimePoint::operator<(const TimePoint& other) const
{
    if(mLowerBound != other.mLowerBound)
    {
        return mLowerBound < other.mLowerBound;
    } else {
        return mUpperBound < other.mUpperBound;
    }
}

TimePoint::Ptr TimePoint::create(const Label& label)
{
    QualitativeTimePoint qt(label);
    return create(qt);
}

TimePoint::Ptr TimePoint::create(const TimePoint& tp)
{
    if(tp.getType() == QUALITATIVE)
    {
        PtrList::iterator cit = std::find_if(msTimePoints.begin(), msTimePoints.end(),
                [tp](const TimePoint::Ptr& otherTp) -> bool
                {
                    return otherTp->getLabel() == tp.getLabel();
                });

        if(cit != msTimePoints.end())
        {
            // return existing shared pointer
            return *cit;
        } else {
            TimePoint::Ptr timepoint = QualitativeTimePoint::getInstance( tp.getLabel() );
            msTimePoints.push_back(timepoint);
            return timepoint;
        }
    } else {
        return create(tp.mLowerBound, tp.mUpperBound);
    }
}

TimePoint::Ptr TimePoint::create(uint64_t lowerBound, uint64_t upperBound)
{
    return TimePoint::Ptr( new TimePoint(lowerBound, upperBound) );
}

std::string TimePoint::toString() const
{
    return toString(0);
}

std::string TimePoint::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    std::string label = getLabel();
    ss << hspace << "Timepoint: [" << mLowerBound << "," << mUpperBound << "]";
    return ss.str();
}

std::string TimePoint::toString(const std::vector<TimePoint::Ptr>& timepoints, uint32_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');

    std::vector<TimePoint::Ptr>::const_iterator cit = timepoints.begin();
    ss << hspace << "Timepoints: [" << std::endl;
    for(; cit != timepoints.end(); ++cit)
    {
        ss << hspace << "    " << (*cit)->toString() << std::endl;
    }
    ss << hspace << "]";
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const TimePointList& timepoints)
{
    os << TimePoint::toString(timepoints);
    return os;
}

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
