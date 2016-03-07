#include "PersistenceCondition.hpp"
#include "Event.hpp"

namespace templ {
namespace solvers {
namespace temporal {

PersistenceCondition::PersistenceCondition(const symbols::StateVariable& stateVariable,
        const Symbol::Ptr& value,
        const point_algebra::TimePoint::Ptr& fromTimepoint,
        const point_algebra::TimePoint::Ptr& toTimepoint)
        : TemporalAssertion(stateVariable, TemporalAssertion::PERSISTENCE_CONDITION)
        , mpValue(value)
        , mpFromTimepoint(fromTimepoint)
        , mpToTimepoint(toTimepoint)
{}

PersistenceCondition::Ptr PersistenceCondition::getInstance(const symbols::StateVariable& stateVariable,
        const Symbol::Ptr& value,
        const point_algebra::TimePoint::Ptr& fromTimepoint,
        const point_algebra::TimePoint::Ptr& toTimepoint)
{
    return PersistenceCondition::Ptr( new PersistenceCondition(stateVariable, value, fromTimepoint, toTimepoint));
}

bool PersistenceCondition::refersToSameValue(const Event::Ptr& other, const point_algebra::TimePointComparator& comparator) const
{
    if(comparator.equals(other->mpTimepoint, mpFromTimepoint))
    {
        // checks if event has established the value that holds in the persistence
        // condition
        return other->mpToValue->equals(mpValue);
    } else if(comparator.equals(other->mpTimepoint, mpToTimepoint))
    {
        // check if event starts with the value that holds in the persistence
        // condition
        return other->mpFromValue->equals(mpValue);
    } else{
        throw std::invalid_argument("templ::PersistenceCondition: Event is disjoint from persistence condition, cannot check for same value");
    }
}

bool PersistenceCondition::refersToSameValue(const shared_ptr<PersistenceCondition>& other, const point_algebra::TimePointComparator& comparator) const
{
    return mpValue->equals(other->mpValue);
}


bool PersistenceCondition::disjointFrom(const shared_ptr<Event>& other, const point_algebra::TimePointComparator& comparator) const
{
    // Checks if timepoint of event is outside of the define interval of the
    // persistence condition
    return comparator.lessThan(other->mpTimepoint, mpFromTimepoint) || comparator.greaterThan(other->mpTimepoint, mpToTimepoint);
}

bool PersistenceCondition::disjointFrom(const shared_ptr<PersistenceCondition>& other, const point_algebra::TimePointComparator& comparator) const
{
    return !comparator.hasIntervalOverlap(mpFromTimepoint, mpToTimepoint, other->mpFromTimepoint, other->mpToTimepoint);
}

std::string PersistenceCondition::toString() const
{
    std::string ss = TemporalAssertion::toString() + "::";
    ss += mpValue->toString();
    ss += "@";
    ss += "[" + mpFromTimepoint->toString() + ",";
    ss += mpToTimepoint->toString() + ")";
    return ss;
}

bool PersistenceCondition::operator==(const PersistenceCondition& other) const
{
    //LOG_DEBUG_S << "type: " << (mType == other.mType) << std::endl
    //    << "state: " << (mStateVariable == other.mStateVariable) << std::endl
    //    << "value: " << mpValue->toString() << " vs. " << other.mpValue->toString() << ":" << (mpValue == other.mpValue) << std::endl
    //    << "from: " << (mpFromTimepoint == other.mpFromTimepoint) << std::endl
    //    << "to: " << (mpToTimepoint == other.mpToTimepoint) << std::endl;
    return mType == other.mType &&
        mStateVariable == other.mStateVariable &&
        mpValue->equals(other.mpValue) &&
        mpFromTimepoint->equals(other.mpFromTimepoint) &&
        mpToTimepoint->equals(other.mpToTimepoint);
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
