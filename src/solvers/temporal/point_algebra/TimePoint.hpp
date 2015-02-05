#ifndef TEREP_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP
#define TEREP_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP

#include <terep/solvers/Variable.hpp>
#include <map>

namespace terep {
namespace solvers {
namespace temporal {
namespace point_algebra {

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

    std::string toString() const;
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace terep
#endif // TEREP_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP
