#include "TimePoint.hpp"

namespace terep {
namespace solvers {
namespace temporal {
namespace point_algebra {

TimePoint::TimePoint(uint64_t lowerBound, uint64_t upperBound)
    : mLowerBound(lowerBound)
    , mUpperBound(upperBound)
{
    if(mUpperBound < mLowerBound)
    {
        std::stringstream ss;
        ss << "TimePoint: upper bound (" << upperBound << ") smaller than lower bound (" << lowerBound << ")";
        throw std::invalid_argument(ss.str());
    }
}

TimePoint::Ptr TimePoint::create(uint64_t lowerBound, uint64_t upperBound)
{
    return TimePoint::Ptr( new TimePoint(lowerBound, upperBound) );
}

std::string TimePoint::toString() const
{
    std::stringstream ss;
    ss << "Timepoint: [" << mLowerBound << "," << mUpperBound << "]";
    return ss.str();
}




} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace terep
