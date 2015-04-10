#include "Event.hpp"
#include "PersistenceCondition.hpp"

namespace templ {

Event::Event()
    : TemporalAssertion(TemporalAssertion::EVENT)
{}

bool Event::refersToSameValue(boost::shared_ptr<Event> other) const
{
    return mFrom == other->mFrom && mTo == other->mTo;
}

bool Event::refersToSameValue(boost::shared_ptr<PersistenceCondition> other) const
{
    return other->refersToSameValue( Event::Ptr(new Event(*this)));
}

} // end namespace templ
