#include "TemporalAssertion.hpp"
#include "Event.hpp"
#include "PersistenceCondition.hpp"
#include <boost/assign/list_of.hpp>

namespace templ {
namespace solvers {
namespace temporal {

TemporalAssertion::TemporalAssertion(const symbols::StateVariable& stateVariable, TemporalAssertion::Type type)
    : mType(type)
    , mStateVariable(stateVariable)
{}

std::map<TemporalAssertion::Type, std::string> TemporalAssertion::TypeTxt = boost::assign::map_list_of
    (UNKNOWN, "UNKNOWN")
    (EVENT, "EVENT")
    (PERSISTENCE_CONDITION, "PERSISTENCE_CONDITION")
    ;

bool TemporalAssertion::isDisjointFrom(const TemporalAssertion::Ptr& other, const point_algebra::TimePointComparator& comparator) const
{
    switch(other->getType())
    {
        case EVENT:
        {
            Event::Ptr event = boost::dynamic_pointer_cast<Event>(other);
            return disjointFrom(event, comparator);
        }
        case PERSISTENCE_CONDITION:
        {
            PersistenceCondition::Ptr pc = boost::dynamic_pointer_cast<PersistenceCondition>(other);
            return disjointFrom(pc, comparator);
        }
        default:
            throw std::invalid_argument("templ::TemporalAssertion::refersToSameValue: cannot handle unknown type");
    }
}

bool TemporalAssertion::isReferringToSameValue(const TemporalAssertion::Ptr& other, const point_algebra::TimePointComparator& comparator) const
{
    switch(other->getType())
    {
        case EVENT:
        {
            Event::Ptr event = boost::dynamic_pointer_cast<Event>(other);
            return refersToSameValue(event, comparator);
        }
        case PERSISTENCE_CONDITION:
        {
            PersistenceCondition::Ptr pc = boost::dynamic_pointer_cast<PersistenceCondition>(other);
            return refersToSameValue(pc, comparator);
        }
        default:
            throw std::invalid_argument("templ::TemporalAssertion::refersToSameValue: cannot handle unknown type");
    }
}

std::string TemporalAssertion::toString() const
{
    return TypeTxt[mType] + "(" + mStateVariable.toString() + ")";
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
