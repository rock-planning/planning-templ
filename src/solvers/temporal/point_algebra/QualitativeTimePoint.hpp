#ifndef TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_QUALITATIVE_TIME_POINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_QUALITATIVE_TIME_POINT_HPP

#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

class QualitativeTimePoint : public TimePoint
{
    std::vector<TimePoint::Label> mAliases;
    TimePoint::Label mLabel;

public:
    QualitativeTimePoint(const TimePoint::Label& label);

    void addAlias(const TimePoint::Label& label);

    bool isAlias(const TimePoint::Label& label) const;

    bool operator==(const QualitativeTimePoint& other) const;

    bool operator!=(const QualitativeTimePoint& other) const { return ! (*this == other); }
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_QUALITATIVE_TIME_POINT_HPP
