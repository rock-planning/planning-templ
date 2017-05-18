#ifndef TEMPL_PERSISTENCE_CONDITION_HPP
#define TEMPL_PERSISTENCE_CONDITION_HPP

#include <templ/Symbol.hpp>
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

    Symbol::Ptr mpValue;

    point_algebra::TimePoint::Ptr mpFromTimepoint;
    point_algebra::TimePoint::Ptr mpToTimepoint;

    bool refersToSameValue(const shared_ptr<Event>& other, const point_algebra::TimePointComparator& comparator) const;
    bool refersToSameValue(const shared_ptr<PersistenceCondition>& other, const point_algebra::TimePointComparator& comparator) const;

    bool disjointFrom(const shared_ptr<Event>& other, const point_algebra::TimePointComparator& comparator) const;
    bool disjointFrom(const shared_ptr<PersistenceCondition>& other, const point_algebra::TimePointComparator& comparator) const;

public:
    typedef shared_ptr<PersistenceCondition> Ptr;

    PersistenceCondition(const symbols::StateVariable& stateVariable,
            const Symbol::Ptr& value,
            const point_algebra::TimePoint::Ptr& fromTimepoint,
            const point_algebra::TimePoint::Ptr& toTimepoint);

    static PersistenceCondition::Ptr getInstance(const symbols::StateVariable& stateVariable,
            const Symbol::Ptr& value,
            const point_algebra::TimePoint::Ptr& fromTimepoint,
            const point_algebra::TimePoint::Ptr& toTimepoint);

    virtual ~PersistenceCondition() {}

    Symbol::Ptr getValue() const { return mpValue; }

    point_algebra::TimePoint::Ptr getFromTimePoint() const { return mpFromTimepoint; }
    point_algebra::TimePoint::Ptr getToTimePoint() const { return mpToTimepoint; }


    virtual std::string toString() const;
    virtual std::string toString(size_t indent) const;

    virtual bool operator==(const PersistenceCondition& other) const;

    /**
     * Get the class name of this constraint
     * \return classname
     */
    virtual std::string getClassName() const { return "PersistenceCondition"; }

protected:
    /// Make sure cloning works
    virtual graph_analysis::Vertex* getClone() const { return new PersistenceCondition(*this); }
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_PERSISTENCE_CONDITION_HPP
