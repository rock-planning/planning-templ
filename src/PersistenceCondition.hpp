#ifndef TEMPL_PERSISTENCE_CONDITION_HPP
#define TEMPL_PERSISTENCE_CONDITION_HPP

#include <templ/Value.hpp>
#include <templ/Timepoint.hpp>
#include <templ/TemporalAssertion.hpp>

namespace templ {

class Event;

/**
 * Specifies that a value persists and being equal to a given 
 * value over an time interval
 * \see "Automated Planning - Theory and Practice" p. 329
 */
class PersistenceCondition : public TemporalAssertion
{
    Value mValue;

    Timepoint mFrom;
    Timepoint mTo;

public:
    typedef boost::shared_ptr<PersistenceCondition> Ptr;

    PersistenceCondition();

    bool refersToSameValue(boost::shared_ptr<Event> other) const;
    bool refersToSameValue(boost::shared_ptr<PersistenceCondition> other) const;

};

} // end namespace templ
#endif // TEMPL_PERSISTENCE_CONDITION_HPP
