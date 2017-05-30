#ifndef TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP

#include <map>
#include <templ/solvers/Variable.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/shared_ptr.hpp>

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

    typedef shared_ptr<TimePoint> Ptr;
    typedef std::string Label;
    typedef std::vector<Label> LabelList;
    typedef std::vector<TimePoint::Ptr> PtrList;

protected:
    TimePoint(uint64_t lowerBound, uint64_t upperBound, Type type);

public:
    friend class boost::serialization::access;

    /**
     * Add default constructor to allow for serialization
     */
    TimePoint();

    TimePoint(uint64_t lowerBound, uint64_t upperBound);

    virtual ~TimePoint() {}

    Type getType() const { return mType; }

    /**
     * Creates a new timepoint reference
     */
    static TimePoint::Ptr create(const TimePoint& t);

    /**
     * Creates a new (qualitative) TimePoint
     */
    static TimePoint::Ptr create(const Label& label);

    /**
     * Create a new (quantitative) TimePoint
     */
    static TimePoint::Ptr create(uint64_t lowerBound, uint64_t upperBound);

    virtual bool equals(const TimePoint::Ptr& other) const;

    virtual bool operator==(const TimePoint& other) const { return mLowerBound == other.mLowerBound && mUpperBound == other.mUpperBound; }
    bool operator<(const TimePoint& other) const;

    void setLowerBound(uint64_t bound) { mLowerBound = bound; }
    void setUpperBound(uint64_t bound) { mUpperBound = bound; }

    uint64_t getLowerBound() const { return mLowerBound; }
    uint64_t getUpperBound() const { return mUpperBound; }

    virtual std::string toString() const;

    virtual std::string toString(uint32_t indent) const;

    /**
     * Create string representation for timepoint list
     * \indent number of spaces for indentation
     */
    static std::string toString(const std::vector<TimePoint::Ptr>& timepoints, uint32_t indent = 0);

    /**
     * Check that lower bound is less or equal to upper bound
     * \throw std::invalid_argument if condition does not how
     */
    static void validateBounds(uint64_t lowerBound, uint64_t upperBound);

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mLabel;
        ar & mLowerBound;
        ar & mUpperBound;
        ar & mType;
    }

private:
    uint64_t mLowerBound;
    uint64_t mUpperBound;
    Type mType;

    static TimePoint::PtrList msTimePoints;
};

typedef std::vector<TimePoint::Ptr> TimePointList;

std::ostream& operator<<(std::ostream&, const TimePointList& timepoints);

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_POINT_ALGEBRA_TIME_POINT_HPP
