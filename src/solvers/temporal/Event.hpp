#ifndef TEMPL_EVENT_HPP
#define TEMPL_EVENT_HPP

#include <templ/PlannerElement.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/solvers/temporal/TemporalAssertion.hpp>

namespace templ {
namespace solvers {
namespace temporal {

/**
 * \class Event
 * \brief "An event, denoted as x@t:(v_1,v_2) specifies the instantaneous change of the value of
 * x from v_1 to _v2 at time t, with v_1 != v_2"
 */
class Event : public TemporalAssertion
{
    friend class PersistenceCondition;

protected:
    PlannerElement::Ptr mpFromValue;
    PlannerElement::Ptr mpToValue;
    
    // Time constant or time variable
    point_algebra::TimePoint::Ptr mpTimepoint;

    bool refersToSameValue(boost::shared_ptr<Event> other, const point_algebra::TimePointComparator& comparator) const;
    bool refersToSameValue(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const;

    bool disjointFrom(boost::shared_ptr<Event> other, const point_algebra::TimePointComparator& comparator) const;
    bool disjointFrom(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const;

public:
    typedef boost::shared_ptr<Event> Ptr;

    Event(const StateVariable& stateVariable, PlannerElement::Ptr from, PlannerElement::Ptr to, point_algebra::TimePoint::Ptr timepoint);

    PlannerElement::Ptr getFromValue() const { return mpFromValue; }
    PlannerElement::Ptr getToValue() const { return mpToValue; }

    point_algebra::TimePoint::Ptr getTimePoint() const { return mpTimepoint; }
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_EVENT_HPP
