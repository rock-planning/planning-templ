#include "PersistenceCondition.hpp"
#include "Event.hpp"

namespace templ {

PersistenceCondition::PersistenceCondition()
        : TemporalAssertion(TemporalAssertion::PERSISTENCE_CONDITION)
{}

bool PersistenceCondition::refersToSameValue(Event::Ptr other) const
{
    if(other->mTimepoint == mFrom)
    {
        return other->mTo == mValue;
    } else if(other->mTimepoint == mTo)
    {
        return other->mFrom == mValue;
    } else{
        throw std::invalid_argument("templ::PersistenceCondition: Event is disjoint from persistence condition, cannot check for same value");
    }
}

bool PersistenceCondition::refersToSameValue(boost::shared_ptr<PersistenceCondition> other) const
{
    return mValue == other->mValue;
}

} // end namespace templ
