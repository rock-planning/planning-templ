#ifndef TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP

#include <map>
#include <templ/solvers/Variable.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

/**
 * \class TimePoint as variable part of the point_algebra
 * \see templ::TimePoint
 */
class TimePoint : public Variable
{
    uint64_t mLowerBound;
    uint64_t mUpperBound;

public:
    typedef boost::shared_ptr<TimePoint> Ptr;

    TimePoint(uint64_t lowerBound, uint64_t upperBound);

    static TimePoint::Ptr create(uint64_t lowerBound, uint64_t upperBound);

    bool operator=(const TimePoint& other) const { return mLowerBound == other.mLowerBound && mUpperBound == other.mUpperBound; }

    void setLowerBound(uint64_t bound) { mLowerBound = bound; }
    void setUpperBound(uint64_t bound) { mUpperBound = bound; }

    uint64_t getLowerBound() const { return mLowerBound; }
    uint64_t getUpperBound() const { return mUpperBound; }

    virtual std::string toString() const;
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP
