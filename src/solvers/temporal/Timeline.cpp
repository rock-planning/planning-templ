#include "Timeline.hpp"
#include <numeric/Combinatorics.hpp>

namespace templ {
namespace solvers {
namespace temporal {

Timeline::Timeline(const StateVariable& stateVariable)
    : mStateVariable(stateVariable)
{}

void Timeline::addTemporalAssertion(TemporalAssertion::Ptr assertion)
{
    if(assertion->getStateVariable() != mStateVariable)
    {
        throw std::invalid_argument("templ::solvers::temporal::Timeline::addTemporalAssertion: cannot add TemporalAssertion since it refers to a different StateVariable");
    }
    mTemporalAssertions.push_back(assertion);
}

void Timeline::addConstraint(Constraint::Ptr constraint)
{
    mConstraints.push_back(mConstraints);
}

bool Timeline::isConsistent() const
{
    QualitativeTemporalConstraintNetwork qtcn;
    TimePointComparator comparator(qtcn); 

    numeric::Combination<TemporalAssertion::Ptr> combination(mTemporalAssertions, 2, numeric::EXACT);
    do {
        std::vector<TemporalAssertion::Ptr> assertionPair = combination.current();

        TemporalAssertion::Ptr a0 = assertionPair[0];
        TemporalAssertion::Ptr a1 = assertionPair[1];

        if( !a0->isDisjointFrom(a1, comparator) && !a0->isReferringSameValue(a1, comparator))
        {
            return false;
        }
    } while(combination.next());
}

ConstantList Timeline::getConstants() const
{
}

ObjectVariableList Timeline::getObjectVariables() const
{
}

TimePointList Timeline::getTimePointList() const
{
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
