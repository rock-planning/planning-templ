#include "PersistenceCondition.hpp"
#include "Event.hpp"

namespace templ {

PersistenceCondition::PersistenceCondition(const StateVariable& stateVariable, Value::Ptr value, const Timepoint& fromTimepoint, const Timepoint& toTimepoint)
        : TemporalAssertion(stateVariable, TemporalAssertion::PERSISTENCE_CONDITION)
        , mpValue(value)
        , mFromTimepoint(fromTimepoint)
        , mToTimepoint(toTimepoint)
{}

bool PersistenceCondition::refersToSameValue(Event::Ptr other) const
{
    if(other->mTimepoint == mFromTimepoint)
    {
        return other->mpToValue->equals(mpValue);
    } else if(other->mTimepoint == mToTimepoint)
    {
        return other->mpFromValue->equals(mpValue);
    } else{
        throw std::invalid_argument("templ::PersistenceCondition: Event is disjoint from persistence condition, cannot check for same value");
    }
}

bool PersistenceCondition::refersToSameValue(boost::shared_ptr<PersistenceCondition> other) const
{
    return mpValue == other->mpValue;
}

} // end namespace templ
