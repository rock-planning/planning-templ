#ifndef TEMPL_SOLVERS_INTERVAL_CONSTRAINT_HPP
#define TEMPL_SOLVERS_INTERVAL_CONSTRAINT_HPP

#include <graph_analysis/EdgeRegistration.hpp>
#include "../../constraints/SimpleConstraint.hpp"
#include "point_algebra/TimePoint.hpp"
#include "Bounds.hpp"

#define T_INTERVALCONSTRAINT(x) dynamic_pointer_cast<templ::solvers::IntervalConstraint>(x)

namespace templ {
namespace solvers {
namespace temporal {

/**
 * An IntervalConstraint represents an edge in the ConstraintNetwork an interval
 * constraint containts a list of intervals which represent the constraints for
 * the transition between two TimePoints
 */
class IntervalConstraint : public constraints::SimpleConstraint
{
private:
    std::vector<Bounds> mIntervals;
    static const graph_analysis::EdgeRegistration<IntervalConstraint> msRegistration;

public:

    typedef shared_ptr<IntervalConstraint> Ptr;

    IntervalConstraint();

    /**
     * Default constructor for an interval constraint
     */
    IntervalConstraint(const point_algebra::TimePoint::Ptr& source,
            const point_algebra::TimePoint::Ptr& target);

    virtual ~IntervalConstraint() {}

    /**
     * Get the class name of this constraint
     * \return classname
     */
    virtual std::string getClassName() const override { return "IntervalConstraint"; }

    /**
     * Get stringified object
     * \return string repr
     */
    virtual std::string toString() const override;

    virtual std::string toString(uint32_t indent = 0) const override;

    /**
     * Serialize this object
     * \return serialized data blob
     */
    std::string serializeBounds() const;

    /**
     * Deserialize a blob into this object
     */
    void deserializeBounds(const std::string& blob);

    /**
     * Get the source vertex/TimePoint of this interval constraint
     */
    point_algebra::TimePoint::Ptr getSourceTimePoint() { return dynamic_pointer_cast<point_algebra::TimePoint>( getSourceVertex()); }

    /**
     * Get the target vertex/TimePoint of this interval constraint
     */
    point_algebra::TimePoint::Ptr getTargetTimePoint() { return dynamic_pointer_cast<point_algebra::TimePoint>( getTargetVertex()); }

    /**
     * Add a new interval
     * \param newInterval
     */
    void addInterval(const Bounds& newInterval);

    /**
     * Get the intervals
     * \return list of bounds/intervals
     */
    const std::vector<Bounds>& getIntervals() const { return mIntervals; }

    /**
     * Get the lowest bound
     */
    double getLowerBound() const;

    /**
     * Get upper bound
     */
    double getUpperBound() const;

    /**
     * Get number of intervals of this constraint
     * \returns the number of intervals of an interval constraint
     */
    size_t getIntervalsNumber() const {return mIntervals.size(); }

    /**
     * Check if a given interval is included in an interval constraint
     * \return true if interval is included in this constraint
     */
    bool checkInterval(const Bounds& x);

    /**
     * Register attributes for serialization
     */
    virtual void registerAttributes(graph_analysis::EdgeTypeManager*) const override;

    /**
     * Merge bounds from another interval constraint
     */
    void appendBounds(const IntervalConstraint& other);

protected:

    /// Make sure cloning works for this constraint
    virtual graph_analysis::Edge* getClone() const { return new IntervalConstraint(*this); }

};

typedef std::vector<IntervalConstraint::Ptr> IntervalConstraintList;

} // end namespace temporal
} // end namespace solvers
} // end namespace templ

#endif // TEMPL_SOLVERS_INTERVAL_CONSTRAINT_HPP
