#include "Event.hpp"
#include "PersistenceCondition.hpp"

namespace templ {

Event::Event(const StateVariable& stateVariable, Value::Ptr from, Value::Ptr to, const Timepoint& timepoint)
    : TemporalAssertion(stateVariable, TemporalAssertion::EVENT)
    , mpFromValue(from)
    , mpToValue(to)
    , mTimepoint(timepoint)
{}

bool Event::refersToSameValue(boost::shared_ptr<Event> other, const TimepointComparator& comparator) const
{
    return mpFromValue->equals(other->mpFromValue) && mpToValue->equals(other->mpToValue);
}

bool Event::refersToSameValue(boost::shared_ptr<PersistenceCondition> other, const TimepointComparator& comparator) const
{
    return other->refersToSameValue( Event::Ptr(new Event(*this)), comparator);
}

bool Event::disjointFrom(boost::shared_ptr<Event> other, const TimepointComparator& comparator) const
{
    if(comparator.equals(mTimepoint, mFromTimepoint) || comparator.equals(mTimepoint, mToTimepoint))
    {
        return false;
    } else if(comparator.greaterThan(mTimepoint, mFromTimepoint) && comparator.greaterThan(mTimepoint, mToTimepoint) )
    {
        // lies in the interval
        return false;
    } else {
        // there seems to be no overlap
        return true;
    }
}

bool Event::disjointFrom(boost::shared_ptr<PersistenceCondition> other, const TimepointComparator& comparator) const
{
    return other->disjointFrom( Event::Ptr( new Event(*this)), comparator);
}

} // end namespace templ
