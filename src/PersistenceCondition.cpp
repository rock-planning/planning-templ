#include "PersistenceCondition.hpp"
#include "Event.hpp"

namespace templ {

PersistenceCondition::PersistenceCondition(const StateVariable& stateVariable, Value::Ptr value, const Timepoint& fromTimepoint, const Timepoint& toTimepoint)
        : TemporalAssertion(stateVariable, TemporalAssertion::PERSISTENCE_CONDITION)
        , mpValue(value)
        , mFromTimepoint(fromTimepoint)
        , mToTimepoint(toTimepoint)
{}

bool PersistenceCondition::refersToSameValue(Event::Ptr other, const TimepointComparator& comparator) const
{
    if(comparator.equals(other->mTimepoint, mFromTimepoint))
    {
        return other->mpToValue->equals(mpValue);
    } else if(comparator.equals(other->mTimepoint, mToTimepoint))
    {
        return other->mpFromValue->equals(mpValue);
    } else{
        throw std::invalid_argument("templ::PersistenceCondition: Event is disjoint from persistence condition, cannot check for same value");
    }
}

bool PersistenceCondition::refersToSameValue(boost::shared_ptr<PersistenceCondition> other, const TimepointComparator& comparator) const
{
    return mpValue == other->mpValue;
}


bool PersistenceCondition::disjointFrom(boost::shared_ptr<Event> other, const TimepointComparator& comparator) const
{
    if(other.mTimepoint == mFromTimepoint || other.mTimepoint == mToTimepoint)
    {
        return false;
    } else if(other.mTimepoint > mFromTimepoint && other.mTimepoint < mToTimepoint)
    {
        // lies in the interval
        return false;
    } else {
        // there 
        return true;
    }
}

bool PersistenceCondition::disjointFrom(boost::shared_ptr<PersistenceCondition> other, const TimepointComparator& comparator) const
{
    return !comparator.hasIntervalOverlap(mFromTimepoint, mToTimepoint, other.mFromTimepoint, other.mToTimepoint);
}

} // end namespace templ
