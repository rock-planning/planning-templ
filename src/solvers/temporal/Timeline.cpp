#include "Timeline.hpp"
#include <numeric/Combinatorics.hpp>

#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>

namespace templ {
namespace solvers {
namespace temporal {

Timeline::Timeline()
    : mStateVariable( symbols::StateVariable("",""))
    , mConstraintNetwork(new QualitativeTemporalConstraintNetwork())
{}

Timeline::Timeline(const symbols::StateVariable& stateVariable)
    : mStateVariable(stateVariable)
    , mConstraintNetwork(new QualitativeTemporalConstraintNetwork())
{}

void Timeline::addTemporalAssertion(TemporalAssertion::Ptr assertion)
{
    if(assertion->getStateVariable() != mStateVariable)
    {
        throw std::invalid_argument("templ::solvers::temporal::Timeline::addTemporalAssertion: cannot add TemporalAssertion since it refers to a different StateVariable");
    }
    switch(assertion->getType())
    {
        case TemporalAssertion::EVENT:
        {
            Event::Ptr event = dynamic_pointer_cast<Event>(assertion);
            try {
                mConstraintNetwork->addVariable(event->getTimePoint());
            } catch(const std::runtime_error& e)
            {
                // duplicate
            }
            break;
        }
        case TemporalAssertion::PERSISTENCE_CONDITION:
        {
            PersistenceCondition::Ptr persistenceCondition = dynamic_pointer_cast<PersistenceCondition>(assertion);
            point_algebra::TimePoint::Ptr fromTime = persistenceCondition->getFromTimePoint();
            point_algebra::TimePoint::Ptr toTime = persistenceCondition->getToTimePoint();
            try {
                mConstraintNetwork->addVariable(fromTime);
            } catch(const std::runtime_error& e)
            {
                // duplicate
            }
            try {
                mConstraintNetwork->addVariable(toTime);
            } catch(const std::runtime_error& e)
            {
                // duplicate
            }
            mConstraintNetwork->addQualitativeConstraint(fromTime, toTime, point_algebra::QualitativeTimePointConstraint::LessOrEqual);
            break;
        }
        case TemporalAssertion::UNKNOWN:
            throw std::runtime_error("templ::solvers::temporal::Timelin::addTemporalAssertion: trying to add TemporalAssertion of type UNKNOWN -- internal error, this should never happen");
    }
    mTemporalAssertions.push_back(assertion);
}

void Timeline::addConstraint(Constraint::Ptr constraint)
{
    point_algebra::QualitativeTimePointConstraint::Ptr timepointConstraint = dynamic_pointer_cast<point_algebra::QualitativeTimePointConstraint>(constraint);
    if(timepointConstraint)
    {
        mConstraintNetwork->addConstraint(timepointConstraint);
    } else {
        mConstraints.push_back(constraint);
    }
}

bool Timeline::isConsistent() const
{
    point_algebra::TimePointComparator comparator(mConstraintNetwork);

    if(mTemporalAssertions.size() < 2)
    {
        LOG_DEBUG_S << "Only one assertion, thus timeline is consistent";
        return true;
    }

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

symbols::ConstantList Timeline::getConstants() const
{
    return getTypeInstances<symbols::Constant>(Symbol::CONSTANT);
}

symbols::ObjectVariableList Timeline::getObjectVariables() const
{
    return getTypeInstances<symbols::ObjectVariable>(Symbol::OBJECT_VARIABLE);
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
                Event::Ptr event = dynamic_pointer_cast<Event>(assertion);
                TimePoint::Ptr timepoint = event->getTimePoint();
                timepoints.push_back(timepoint);
                break;
            }
            case TemporalAssertion::PERSISTENCE_CONDITION:
            {
                PersistenceCondition::Ptr persistenceCondition = dynamic_pointer_cast<PersistenceCondition>(assertion);
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

std::string Timeline::toString() const
{
    std::stringstream ss;
    ss << "BEGIN Timeline ----" << std::endl;;
    ss << "    StateVariable: " << mStateVariable.toString() << std::endl;
    {
        ss << "    TemporalAssertions: " << std::endl;
        TemporalAssertionList::const_iterator cit = mTemporalAssertions.begin();
        for(; cit != mTemporalAssertions.end(); ++cit)
        {
            ss << "        " << (*cit)->toString() << std::endl;
        }
    }

    {
        ss << "    Constraints: " << std::endl;
        ConstraintList::const_iterator cit = mConstraints.begin();
        for(; cit != mConstraints.end(); ++cit)
        {
            ss << "        " << (*cit)->toString() << std::endl;
        }

        using namespace graph_analysis;
        EdgeIterator::Ptr edgeIterator = mConstraintNetwork->getConstraintIterator();
        while(edgeIterator->next())
        {
            Edge::Ptr edge = edgeIterator->current();
            ss << "        " << edge->toString() << std::endl;
        }
    }
    ss << "END Timeline ----" << std::endl;
    return ss.str();
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
