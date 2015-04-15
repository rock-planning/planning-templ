#ifndef TEMPL_TIMEPOINT_COMPARATOR_HPP
#define TEMPL_TIMEPOINT_COMPARATOR_HPP

#include <map>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

/**
 * \class TimePointComparator
 * \brief This class allows to implement a custom TimePointComparator, e.g.,
 * to account for an existing set of qualitative / quantitative constraints
 */
class TimePointComparator
{
    QualitativeTemporalConstraintNetwork::Ptr mpTemporalConstraintNetwork;

public:
    TimePointComparator(QualitativeTemporalConstraintNetwork::Ptr tcn = QualitativeTemporalConstraintNetwork::Ptr());

    /**
     * Check equality of two timepoints
     */
    virtual bool equals(TimePoint::Ptr t0, TimePoint::Ptr t1) const;

    /**
     * Check if one timepoint is greater (more progressed in time) than another
     */
    virtual bool greaterThan(TimePoint::Ptr t0, TimePoint::Ptr t1) const;

    bool greaterOrEqual(TimePoint::Ptr t0, TimePoint::Ptr t1) const { return equals(t0,t1) || greaterThan(t0, t1); }

    bool lessOrEqual(TimePoint::Ptr t0, TimePoint::Ptr t1) const { return greaterOrEqual(t1, t0); }

    bool lessThan(TimePoint::Ptr t0, TimePoint::Ptr t1) const { return greaterThan(t1,t0); }

    bool hasIntervalOverlap(TimePoint::Ptr a_start, TimePoint::Ptr a_end, TimePoint::Ptr b_start, TimePoint::Ptr b_end) const;

    bool inInterval(TimePoint::Ptr t0, TimePoint::Ptr i_start, TimePoint::Ptr i_end) const;
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_TIMEPOINT_COMPARATOR_HPP
