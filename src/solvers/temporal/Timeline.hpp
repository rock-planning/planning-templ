#ifndef TEMPL_TIMELINE_HPP
#define TEMPL_TIMELINE_HPP

#include <templ/Constant.hpp>
#include <templ/ObjectVariable.hpp>
#include <templ/solvers/Constraint.hpp>
#include <templ/solvers/temporal/TemporalAssertion.hpp>

namespace templ {
namespace solvers {
namespace temporal {

/**
 * \class Timeline
 * \brief A timeline is a chronicle for a single state variable x
 * It serves as partial specification of a function that gives
 * the value of the state variable over time
 *
 * \see Chronicle
 */
class Timeline
{
    StateVariable mStateVariable;

    TemporalAssertionList mTemporalAssertions;
    ConstraintList mConstraints;

public:
    Timeline(const StateVariable& stateVariable);

    /**
     * Add temporal assertion
     */
    void addTemporalAssertion(TemporalAssertion::Ptr assertion);

    /**
     * Add constraint to this timeline
     */
    void addConstraint(Constraint::Ptr constraint);

    /**
     * Check if timeline is consistent, i.e. (Automated Planning p 331 Def.
     * 14.8)
     * - no pair of assertions in the timeline conflicts
     *   i.e. no specification leaves two possibly distinct values of the same
     *   state variable at the same time
     * - every pair of assertions is either disjoint or refers to the same value
     *   and/or the same timepoint
     *
     */
    bool isConsistent() const;

    /**
     * Get all constants that are used within this timeline
     * \return list of constants
     */
    ConstantList getConstants() const;

    /**
     * Get all object variable that are used within this timeline
     * \return list of object variables
     */
    ObjectVariableList getObjectVariables() const;

    /**
     * Get the list of (qualitative) timepoints
     */
    point_algebra::TimePointList getTimePoints() const;
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_TIMELINE_HPP
