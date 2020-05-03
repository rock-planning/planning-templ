#include "TimePoint.hpp"
#include "QualitativeTimePoint.hpp"
#include <sstream>
#include <algorithm>
#include <limits>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

TimePoint::PtrList TimePoint::msTimePoints;

std::map<TimePoint::Type, std::string> TimePoint::TypeTxt = {
    { TimePoint::UNKNOWN, "unknown" },
    { TimePoint::QUALITATIVE, "qualitative" },
    { TimePoint::QUANTITATIVE, "quantitative" }
};

TimePoint::TimePoint(const Label& label,
        uint64_t lowerBound, uint64_t upperBound,
        Type type)
    : Variable(label)
    , mLowerBound(lowerBound)
    , mUpperBound(upperBound)
    , mType(type)
{
    TimePoint::validateBounds(mLowerBound, mUpperBound);
}

TimePoint::TimePoint(const Label& label)
    : TimePoint(label, 0, std::numeric_limits<uint64_t>::max(), QUANTITATIVE)
{}

TimePoint::TimePoint(uint64_t lowerBound, uint64_t upperBound, Type type)
    : TimePoint("", lowerBound, upperBound, type)
{
}

TimePoint::TimePoint(uint64_t lowerBound, uint64_t upperBound)
    : TimePoint("", lowerBound, upperBound, QUANTITATIVE)
{
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
    TimePoint::Ptr t = TimePoint::get(tp.getLabel());
    if(t)
    {
        if(tp.getType() != t->getType())
        {
            throw std::runtime_error("templ::solvers::temporal::TimePoint::create"
                    " type mismatch: a timepoint with label '" + tp.getLabel() + "' already exists"
                    ", but as '" + TimePoint::TypeTxt[t->getType()] + "'");
        } else {
            return t;
        }
    } else {
        switch(tp.getType())
        {
            case QUALITATIVE:
            {
                TimePoint::Ptr timepoint = QualitativeTimePoint::getInstance( tp.getLabel() );
                msTimePoints.push_back(timepoint);
                return timepoint;
            }
            case QUANTITATIVE:
                return create(tp.mLowerBound, tp.mUpperBound,tp.getLabel());
            default:
                break;
        }
    }
}

TimePoint::Ptr TimePoint::get(const Label& label)
{
    if(label.empty())
    {
        return TimePoint::Ptr();
    }

    PtrList::iterator cit = std::find_if(msTimePoints.begin(), msTimePoints.end(),
            [label](const TimePoint::Ptr& otherTp) -> bool
            {
                return otherTp->getLabel() == label;
            });
    if(cit != msTimePoints.end())
    {
        return *cit;
    } else {
        return TimePoint::Ptr();
    }
}

TimePoint::Ptr TimePoint::create(uint64_t lowerBound, uint64_t upperBound,
        const Label& label)
{
    TimePoint::Ptr t = make_shared<TimePoint>(label, lowerBound, upperBound);
    msTimePoints.push_back(t);
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
