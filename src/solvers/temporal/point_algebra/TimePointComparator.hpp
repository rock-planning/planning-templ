#ifndef TEMPL_TIMEPOINT_COMPARATOR_HPP
#define TEMPL_TIMEPOINT_COMPARATOR_HPP

#include <map>
#include "TimePoint.hpp"
#include "../TemporalConstraintNetwork.hpp"

namespace templ {
namespace solvers {
namespace temporal {

    class Interval;

namespace point_algebra {

/**
 * \class TimePointComparator
 * \brief This class allows to implement a custom TimePointComparator, e.g.,
 * to account for an existing set of qualitative / quantitative constraints
 */
class TimePointComparator
{
    TemporalConstraintNetwork::Ptr mpTemporalConstraintNetwork;

public:
    /**
     * Default constructor
     * Note that to handle qualitative constraints -- a
     * QualitativeTemporalConstraintNetwork needs to be given
     */
    TimePointComparator(const TemporalConstraintNetwork::Ptr& tcn = TemporalConstraintNetwork::Ptr());

    virtual ~TimePointComparator() {};

    /**
     * Get the temporal constraint network
     * \return Associated temporal constraint network
     */
    TemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() const { return mpTemporalConstraintNetwork; }

    /**
     * Sort a list of timepoints based on this timepoint comparator (using the
     * lessThan function
     */
    void sort(std::vector<point_algebra::TimePoint::Ptr>& timepoints) const;

    /**
     * Check equality of two timepoints
     */
    virtual bool equals(const TimePoint::Ptr& t0, const TimePoint::Ptr& t1) const;

    /**
     * Check if one timepoint (t0) is greater (more progressed in time) than another (t1)
     */
    virtual bool greaterThan(const TimePoint::Ptr& t0, const TimePoint::Ptr& t1) const;

    bool greaterOrEqual(const TimePoint::Ptr t0, const TimePoint::Ptr& t1) const { return equals(t0,t1) || greaterThan(t0, t1); }

    bool lessOrEqual(const TimePoint::Ptr& t0, const TimePoint::Ptr& t1) const { return greaterOrEqual(t1, t0); }

    bool lessThan(const TimePoint::Ptr& t0, const TimePoint::Ptr& t1) const { return greaterThan(t1,t0); }

    bool hasIntervalOverlap(const TimePoint::Ptr& a_start, const TimePoint::Ptr& a_end, const TimePoint::Ptr& b_start, const TimePoint::Ptr& b_end) const;

    Interval getIntervalOverlap(const TimePoint::Ptr& a_start, const TimePoint::Ptr& a_end, const TimePoint::Ptr& b_start, const TimePoint::Ptr& b_end) const;

    bool inInterval(const TimePoint::Ptr& t0, const TimePoint::Ptr& i_start, const TimePoint::Ptr& i_end) const;
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_TIMEPOINT_COMPARATOR_HPP
