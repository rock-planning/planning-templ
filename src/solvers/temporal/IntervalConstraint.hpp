#ifndef TEMPL_SOLVERS_INTERVAL_CONSTRAINT_HPP
#define TEMPL_SOLVERS_INTERVAL_CONSTRAINT_HPP

#include <graph_analysis/Edge.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/solvers/temporal/Bounds.hpp>

#define T_INTERVALCONSTRAINT(x) boost::dynamic_pointer_cast<templ::solvers::IntervalConstraint>(x)

namespace templ {
namespace solvers {
namespace temporal {

/**
 * An Interval Constraint represents an edge in the constraint network identified by a set of intervals
 * \ which represents the constraints between two TimePoints
 */
class IntervalConstraint : public graph_analysis::Edge
{
private:
    std::vector<Bounds> intervals;

public:

    typedef boost::shared_ptr<IntervalConstraint> Ptr;

    /**
     * Default constructor for an interval constraint
     */
    IntervalConstraint(point_algebra::TimePoint::Ptr source, point_algebra::TimePoint::Ptr target);

    virtual ~IntervalConstraint() {}

    /**
     * Get the class name of this constraint
     * \return classname
     */
    virtual std::string getClassName() const { return "IntervalConstraint"; }

    /**
     * Get stringified object
     * \return string repr
     */
    virtual std::string toString() const;

    /**
     * Get the source vertex/TimePoint of this interval constraint
     */
    point_algebra::TimePoint::Ptr getSourceTimePoint() { return boost::dynamic_pointer_cast<point_algebra::TimePoint>( getSourceVertex()); }

    /**
     * Get the target vertex/TimePoint of this interval constraint
     */
    point_algebra::TimePoint::Ptr getTargetTimePoint() { return boost::dynamic_pointer_cast<point_algebra::TimePoint>( getTargetVertex()); }

    // add a new interval
    void addInterval(Bounds newInterval) { intervals.push_back(newInterval); }

    // returns the set of intervals for an interval constraint
    std::vector<Bounds> getIntervals() { return intervals; }

    // returns the number of intervals of an interval constraint
    int getIntervalsNumber() {return intervals.size(); }

    // check if a given interval is included in an interval constraint
    bool checkInterval(Bounds x);

protected:

    /// Make sure cloning works for this constraint
    virtual graph_analysis::Edge* doClone() { return new IntervalConstraint(*this); }

};

typedef std::vector<IntervalConstraint::Ptr> IntervalConstraintList;

} // end namespace temporal
} // end namespace solvers
} // end namespace templ

#endif // TEMPL_SOLVERS_INTERVAL_CONSTRAINT_HPP
