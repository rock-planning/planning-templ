#include "Timeline.hpp"
#include <numeric/Combinatorics.hpp>

#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>

namespace templ {
namespace solvers {
namespace temporal {

Timeline::Timeline()
    : mStateVariable(StateVariable("",""))
{}

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
    mConstraints.push_back(constraint);
}

bool Timeline::isConsistent() const
{
    QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());
    point_algebra::TimePointComparator comparator(qtcn);

    numeric::Combination<TemporalAssertion::Ptr> combination(mTemporalAssertions, 2, numeric::EXACT);
    do {
        std::vector<TemporalAssertion::Ptr> assertionPair = combination.current();

        TemporalAssertion::Ptr a0 = assertionPair[0];
        TemporalAssertion::Ptr a1 = assertionPair[1];

        if( !a0->isDisjointFrom(a1, comparator) && !a0->isReferringToSameValue(a1, comparator))
        {
            return false;
        }
    } while(combination.next());
    return true;
}

ConstantList Timeline::getConstants() const
{
    return getTypeInstances<Constant>(PlannerElement::CONSTANT);
}

ObjectVariableList Timeline::getObjectVariables() const
{
    return getTypeInstances<ObjectVariable>(PlannerElement::OBJECT_VARIABLE);
}

point_algebra::TimePointList Timeline::getTimePoints() const
{
    using namespace templ::solvers::temporal::point_algebra;

    TimePointList timepoints;

    TemporalAssertionList::const_iterator cit = mTemporalAssertions.begin();
    for(; cit != mTemporalAssertions.end(); ++cit)
    {
        TemporalAssertion::Ptr assertion = *cit;
        switch(assertion->getType())
        {
            case TemporalAssertion::EVENT:
            {
                Event::Ptr event = boost::dynamic_pointer_cast<Event>(assertion);
                TimePoint::Ptr timepoint = event->getTimePoint();
                timepoints.push_back(timepoint);
                break;
            }
            case TemporalAssertion::PERSISTENCE_CONDITION:
            {
                PersistenceCondition::Ptr persistenceCondition = boost::dynamic_pointer_cast<PersistenceCondition>(assertion);
                TimePoint::Ptr fromTime = persistenceCondition->getFromTimePoint();
                timepoints.push_back(fromTime);

                TimePoint::Ptr toTime = persistenceCondition->getFromTimePoint();
                timepoints.push_back(toTime);
                break;
            }
            default:
                throw std::runtime_error("templ::solvers::temporal::Timeline::getTimePoints: hit TemporalAssertion of type UNKNOWN -- this is an internal error and should never happend");
        }
    }
    return timepoints;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
