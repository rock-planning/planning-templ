#ifndef TEMPL_PERSISTENCE_CONDITION_HPP
#define TEMPL_PERSISTENCE_CONDITION_HPP

#include <templ/Value.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/solvers/temporal/TemporalAssertion.hpp>

namespace templ {
namespace solvers {
namespace temporal {

class Event;

/**
 * \class PersistenceCondition
 * \brief Specifies that a value persists and being equal to a given 
 * value over an time interval
 * \see "Automated Planning - Theory and Practice" p. 329
 */
class PersistenceCondition : public TemporalAssertion
{
    friend class Event;

    Value::Ptr mpValue;

    point_algebra::TimePoint::Ptr mpFromTimepoint;
    point_algebra::TimePoint::Ptr mpToTimepoint;

    bool refersToSameValue(boost::shared_ptr<Event> other, const point_algebra::TimePointComparator& comparator) const;
    bool refersToSameValue(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const;

    bool disjointFrom(boost::shared_ptr<Event> other, const point_algebra::TimePointComparator& comparator) const;
    bool disjointFrom(boost::shared_ptr<PersistenceCondition> other, const point_algebra::TimePointComparator& comparator) const;

public:
    typedef boost::shared_ptr<PersistenceCondition> Ptr;

    PersistenceCondition(const StateVariable& stateVariable, Value::Ptr value, point_algebra::TimePoint::Ptr fromTimepoint, point_algebra::TimePoint::Ptr toTimepoint);

};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_PERSISTENCE_CONDITION_HPP
