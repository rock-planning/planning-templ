#ifndef TEMPL_EVENT_HPP
#define TEMPL_EVENT_HPP

#include <templ/Value.hpp>
#include <templ/Timepoint.hpp>
#include <templ/TemporalAssertion.hpp>

namespace templ {

/**
 * "An event, denoted as x@t:(v_1,v_2) specifies the instantaneous change of the value of
 * x from v_1 to _v2 at time t, with v_1 != v_2
 */
class Event : public TemporalAssertion
{
    friend class PersistenceCondition;

protected:
    Value::Ptr mpFromValue;
    Value::Ptr mpToValue;
    
    // Time constant or time variable
    Timepoint mTimepoint;
public:
    typedef boost::shared_ptr<Event> Ptr;

    Event(const StateVariable& stateVariable, Value::Ptr from, Value::Ptr to, const Timepoint& timepoint);

    bool refersToSameValue(boost::shared_ptr<Event> other) const;
    bool refersToSameValue(boost::shared_ptr<PersistenceCondition> other) const;
};

} // end namespace templ
#endif // TEMPL_EVENT_HPP
