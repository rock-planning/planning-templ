#ifndef TEMPL_CHRONICLE_HPP
#define TEMPL_CHRONICLE_HPP

#include <map>
#include <templ/StateVariable.hpp>
#include <templ/solvers/temporal/Timeline.hpp>

namespace templ {
namespace solvers {
namespace temporal {

/**
 * \class Chronicle
 * \brief "A chronicle for a set of state variables {x_i,...x_j} is a pair
 * phi = (F,C), where F is a set of temporal assertions, i.e., events and
 * persistence conditions about the state variables x_i,...,x_j, and C
 * is a set of object constraints and temporal constraints."
 * (Automated Planning -- Theory and Practice)
 *
 * \see Timeline
 */
class Chronicle
{
    std::map<StateVariable, Timeline> mTimelines;

public:
    /**
     * Add a timeline
     */
    void addTimeline(const Timeline& timeline);

    /**
     * Check if chronicle is consistent
     * Iterates over all timelines to check if they are consistent
     */
    bool isConsistent() const;

    /**
     * Check if chronicle supports another chronicle
     */
    bool isSupporting(const Chronicle& other) const;

    /**
     * Stringify Chronicle
     */
    std::string toString() const;
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_CHRONICLE_HPP
