#include "Event.hpp"
#include "PersistenceCondition.hpp"

namespace templ {

Event::Event(const StateVariable& stateVariable, Value::Ptr from, Value::Ptr to, const Timepoint& timepoint)
    : TemporalAssertion(stateVariable, TemporalAssertion::EVENT)
    , mpFromValue(from)
    , mpToValue(to)
    , mTimepoint(timepoint)
{}

bool Event::refersToSameValue(boost::shared_ptr<Event> other) const
{
    return mpFromValue->equals(other->mpFromValue) && mpToValue->equals(other->mpToValue);
}

bool Event::refersToSameValue(boost::shared_ptr<PersistenceCondition> other) const
{
    return other->refersToSameValue( Event::Ptr(new Event(*this)));
}

} // end namespace templ
