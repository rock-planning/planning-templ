#ifndef TEMPL_TEMPORAL_ASSERTION_HPP
#define TEMPL_TEMPORAL_ASSERTION_HPP

#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <templ/StateVariable.hpp>

namespace templ {

class Event;
class PersistenceCondition;

class TemporalAssertion
{
public:
    enum Type { UNKNOWN = 0,
        EVENT,
        PERSISTENCE_CONDITION
    };

    static std::map<Type, std::string> TypeTxt;

protected:
    /**
     * Construct temporal assertion type
     * Only possible via subclasses
     */
    TemporalAssertion(Type type);

public:
    typedef boost::shared_ptr<TemporalAssertion> Ptr;

    /**
     * Get the type of the temporal assertion
     */
    Type getType() const { return mType; }

    const StateVariable& getStateVariable() const { return mStateVariable; }

    /**
     * Check that TemporalAssertion is disjoint from another
     */
    bool isDisjointFrom(TemporalAssertion::Ptr other) const;

    /**
     * Refers to the same value and/or same timepoint
     * \throw std::invalid_argument if invalid type is used
     */
    bool refersToSameValue(TemporalAssertion::Ptr other) const;

    virtual bool refersToSameValue(boost::shared_ptr<Event> other) const { throw std::runtime_error("templ::TemporalAssertion::refersToSameValue: not implemented"); }

    virtual bool refersToSameValue(boost::shared_ptr<PersistenceCondition> other) const { throw std::runtime_error("templ::TemporalAssertion::refersToSameValue: not implemented"); }
private:
    Type mType;
    StateVariable mStateVariable;

};

typedef std::vector<TemporalAssertion::Ptr> TemporalAssertionList;

} // end namespace templ
#endif // TEMPL_TEMPORAL_ASSERTION_HPP
