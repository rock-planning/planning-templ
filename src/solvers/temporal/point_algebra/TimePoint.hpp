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
public:
    enum Type { UNKNOWN, QUANTITATIVE, QUALITATIVE };

    typedef boost::shared_ptr<TimePoint> Ptr;
    typedef std::string Label;
    typedef std::vector<Label> LabelList;

protected:
    TimePoint(uint64_t lowerBound, uint64_t upperBound, Type type);

public:
    TimePoint(uint64_t lowerBound, uint64_t upperBound);

    virtual ~TimePoint() {}

    Type getType() const { return mType; }

    /**
     * Creates a new (qualitative) TimePoint
     */
    static TimePoint::Ptr create(const Label& label);

    /**
     * Create a new (quantitative) TimePoint
     */
    static TimePoint::Ptr create(uint64_t lowerBound, uint64_t upperBound);

    virtual bool equals(TimePoint::Ptr other) const;

    bool operator==(const TimePoint& other) const { return mLowerBound == other.mLowerBound && mUpperBound == other.mUpperBound; }

    void setLowerBound(uint64_t bound) { mLowerBound = bound; }
    void setUpperBound(uint64_t bound) { mUpperBound = bound; }

    uint64_t getLowerBound() const { return mLowerBound; }
    uint64_t getUpperBound() const { return mUpperBound; }

    virtual std::string toString() const;

    /**
     * Check that lower bound is less or equal to upper bound
     * \throw std::invalid_argument if condition does not how
     */
    static void validateBounds(uint64_t lowerBound, uint64_t upperBound);


private:
    uint64_t mLowerBound;
    uint64_t mUpperBound;
    Type mType;
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP
