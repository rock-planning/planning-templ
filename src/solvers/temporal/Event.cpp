#include "Event.hpp"
#include "PersistenceCondition.hpp"

namespace templ {
namespace solvers {
namespace temporal {

Event::Event(const StateVariable& stateVariable, PlannerElement::Ptr from, PlannerElement::Ptr to, const point_algebra::TimePoint::Ptr timepoint)
    : TemporalAssertion(stateVariable, TemporalAssertion::EVENT)
    , mpFromValue(from)
    , mpToValue(to)
    , mpTimepoint(timepoint)
{}

Event::Ptr Event::getInstance(const StateVariable& stateVariable, PlannerElement::Ptr from, PlannerElement::Ptr to, const point_algebra::TimePoint::Ptr timepoint)
{
    return Event::Ptr( new Event(stateVariable, from, to, timepoint));
}


bool Event::refersToSameValue(boost::shared_ptr<Event> other, const point_algebra::TimePointComparator& comparator) const
{
    return mpFromValue->equals(other->mpFromValue) && mpToValue->equals(other->mpToValue);
}

bool Event::refersToSameValue(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const
{
    return other->refersToSameValue( Event::Ptr(new Event(*this)), comparator);
}

bool Event::disjointFrom(boost::shared_ptr<Event> other, const point_algebra::TimePointComparator& comparator) const
{
    return !comparator.equals(mpTimepoint, other->mpTimepoint);
}

bool Event::disjointFrom(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const
{
    return other->disjointFrom( Event::Ptr( new Event(*this)), comparator);
}

std::string Event::toString() const
{
    std::string ss = TemporalAssertion::toString() + "::";
    ss += "@";
    ss += mpTimepoint->toString() + ":";
    ss += "(";
    ss += mpFromValue->toString() + ",";
    ss += mpToValue->toString() + ")";
    return ss;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ