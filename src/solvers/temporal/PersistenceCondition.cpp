#include "PersistenceCondition.hpp"
#include "Event.hpp"

namespace templ {
namespace solvers {
namespace temporal {

PersistenceCondition::PersistenceCondition(const StateVariable& stateVariable, Value::Ptr value, point_algebra::TimePoint::Ptr frompTimepoint, point_algebra::TimePoint::Ptr toTimepoint)
        : TemporalAssertion(stateVariable, TemporalAssertion::PERSISTENCE_CONDITION)
        , mpValue(value)
        , mpFromTimepoint(frompTimepoint)
        , mpToTimepoint(toTimepoint)
{}

bool PersistenceCondition::refersToSameValue(Event::Ptr other, const point_algebra::TimePointComparator& comparator) const
{
    if(comparator.equals(other->mpTimepoint, mpFromTimepoint))
    {
        return other->mpToValue->equals(mpValue);
    } else if(comparator.equals(other->mpTimepoint, mpToTimepoint))
    {
        return other->mpFromValue->equals(mpValue);
    } else{
        throw std::invalid_argument("templ::PersistenceCondition: Event is disjoint from persistence condition, cannot check for same value");
    }
}

bool PersistenceCondition::refersToSameValue(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const
{
    return mpValue == other->mpValue;
}


bool PersistenceCondition::disjointFrom(boost::shared_ptr<Event> other, const point_algebra::TimePointComparator& comparator) const
{
    if( comparator.greaterOrEqual(other->mpTimepoint, mpFromTimepoint) && comparator.lessOrEqual(other->mpTimepoint, mpToTimepoint) )
    {
        // lies in the interval
        return false;
    } else {
        // there 
        return true;
    }
}

bool PersistenceCondition::disjointFrom(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const
{
    return !comparator.hasIntervalOverlap(mpFromTimepoint, mpToTimepoint, other->mpFromTimepoint, other->mpToTimepoint);
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
