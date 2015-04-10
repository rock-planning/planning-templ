#include "TemporalAssertion.hpp"
#include "Event.hpp"
#include "PersistenceCondition.hpp"

namespace templ {

TemporalAssertion::TemporalAssertion(const StateVariable& stateVariable, TemporalAssertion::Type type)
    : mStateVariable(stateVariable)
    , mType(type)
{}

bool TemporalAssertion::isDisjointFrom(TemporalAssertion::Ptr other) const
{
    throw std::runtime_error("templ::TemporalAssertion::isDisjointFrom: not implemented");
}

bool TemporalAssertion::refersToSameValue(TemporalAssertion::Ptr other) const
{
    switch(other->getType())
    {
        case EVENT:
        {
            Event::Ptr event = boost::dynamic_pointer_cast<Event>(other);
            return refersToSameValue(event);
        }
        case PERSISTENCE_CONDITION:
        {
            PersistenceCondition::Ptr pc = boost::dynamic_pointer_cast<PersistenceCondition>(other);
            return refersToSameValue(pc);
        }
        default:
            throw std::invalid_argument("templ::TemporalAssertion::refersToSameValue: cannot handle unknown type");
    }
}

} // end namespace templ
