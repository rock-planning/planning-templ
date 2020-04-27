#ifndef TEMPL_EVENT_HPP
#define TEMPL_EVENT_HPP

#include "../../Symbol.hpp"
#include "point_algebra/TimePoint.hpp"
#include "TemporalAssertion.hpp"

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
    Symbol::Ptr mpFromValue;
    Symbol::Ptr mpToValue;

    // Time constant or time variable
    point_algebra::TimePoint::Ptr mpTimepoint;

    bool refersToSameValue(const shared_ptr<Event>& other, const point_algebra::TimePointComparator& comparator) const;
    bool refersToSameValue(const shared_ptr<PersistenceCondition>& other, const point_algebra::TimePointComparator& comparator) const;

    bool disjointFrom(const shared_ptr<Event>& other, const point_algebra::TimePointComparator& comparator) const;
    bool disjointFrom(const shared_ptr<PersistenceCondition>& other, const point_algebra::TimePointComparator& comparator) const;

public:
    typedef shared_ptr<Event> Ptr;

    static Event::Ptr getInstance(const symbols::StateVariable& stateVariable,
            const Symbol::Ptr& from,
            const Symbol::Ptr& to,
            const point_algebra::TimePoint::Ptr& timepoint);

    Event(const symbols::StateVariable& stateVariable,
            const Symbol::Ptr& from,
            const Symbol::Ptr& to,
            const point_algebra::TimePoint::Ptr& timepoint);

    virtual ~Event() {}

    Symbol::Ptr getFromValue() const { return mpFromValue; }
    Symbol::Ptr getToValue() const { return mpToValue; }

    point_algebra::TimePoint::Ptr getTimePoint() const { return mpTimepoint; }

    virtual std::string toString() const;

    /**
     * Get the class name of this constraint
     * \return classname
     */
    virtual std::string getClassName() const { return "Event"; }

protected:
    /// Make sure cloning works
    virtual graph_analysis::Vertex* getClone() const { return new Event(*this); }


};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_EVENT_HPP
