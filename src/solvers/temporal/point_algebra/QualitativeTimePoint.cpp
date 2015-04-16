#include "QualitativeTimePoint.hpp"
#include <algorithm>
#include <base/Logging.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

QualitativeTimePoint::QualitativeTimePoint(const TimePoint::Label& label)
    : TimePoint(0,0, QUALITATIVE)
    , mLabel(label)
{
    // also add label as alias, since that will make comparing via set operation
    // possible
    addAlias(label);
}

void QualitativeTimePoint::addAlias(const TimePoint::Label& label)
{
    if(!isAlias(label))
    {
        mAliases.push_back(label);
        std::sort(mAliases.begin(), mAliases.end());
    } else {
        LOG_DEBUG_S << "Alias '" << label << "' already exists for TimePoint '" << mLabel << "'";
    }
}

bool QualitativeTimePoint::isAlias(const TimePoint::Label& label) const
{
    std::vector<TimePoint::Label>::const_iterator cit = std::find(mAliases.begin(), mAliases.end(), label);
    return cit != mAliases.end();
}

bool QualitativeTimePoint::operator==(const QualitativeTimePoint& other) const
{
    // since addAlias does sorting, no need to sort here

    // result
    std::vector<TimePoint::Label> intersection(other.mAliases.size() + mAliases.size());
    std::vector<TimePoint::Label>::iterator it;
    it = std::set_intersection(mAliases.begin(), mAliases.end(), other.mAliases.begin(), other.mAliases.end(), intersection.begin());
    intersection.resize(it-intersection.begin());

    return !intersection.empty();
}

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
