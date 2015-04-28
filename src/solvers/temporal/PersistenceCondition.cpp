#include "PersistenceCondition.hpp"
#include "Event.hpp"

namespace templ {
namespace solvers {
namespace temporal {

PersistenceCondition::PersistenceCondition(const StateVariable& stateVariable, PlannerElement::Ptr value, point_algebra::TimePoint::Ptr fromTimepoint, point_algebra::TimePoint::Ptr toTimepoint)
        : TemporalAssertion(stateVariable, TemporalAssertion::PERSISTENCE_CONDITION)
        , mpValue(value)
        , mpFromTimepoint(fromTimepoint)
        , mpToTimepoint(toTimepoint)
{}

bool PersistenceCondition::refersToSameValue(Event::Ptr other, const point_algebra::TimePointComparator& comparator) const
{
    if(comparator.equals(other->mpTimepoint, mpFromTimepoint))
    {
        // checks if event has established the value that holds in the persistence
        // condition
        return other->mpToValue->equals(mpValue);
    } else if(comparator.equals(other->mpTimepoint, mpToTimepoint))
    {
        // check if event starts with the value that holds in the persistence
        // condition
        return other->mpFromValue->equals(mpValue);
    } else{
        throw std::invalid_argument("templ::PersistenceCondition: Event is disjoint from persistence condition, cannot check for same value");
    }
}

bool PersistenceCondition::refersToSameValue(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const
{
    return mpValue->equals(other->mpValue);
}


bool PersistenceCondition::disjointFrom(boost::shared_ptr<Event> other, const point_algebra::TimePointComparator& comparator) const
{
    // Checks if timepoint of event is outside of the define interval of the
    // persistence condition
    return comparator.lessThan(other->mpTimepoint, mpFromTimepoint) || comparator.greaterThan(other->mpTimepoint, mpToTimepoint);
}

bool PersistenceCondition::disjointFrom(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const
{
    return !comparator.hasIntervalOverlap(mpFromTimepoint, mpToTimepoint, other->mpFromTimepoint, other->mpToTimepoint);
}

std::string PersistenceCondition::toString() const
{
    std::string ss = TemporalAssertion::toString() + "::";
    ss += mpValue->toString();
    ss += "@";
    ss += "[" + mpFromTimepoint->toString() + ",";
    ss += mpToTimepoint->toString() + ")";
    return ss;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
