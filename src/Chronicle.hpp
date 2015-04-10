#ifndef TEMPL_CHRONICLE_HPP
#define TEMPL_CHRONICLE_HPP

#include <map>
#include <templ/StateVariable.hpp>
#include <templ/Timeline.hpp>

namespace templ {

/**
 * "A chronicle for a set of state variables {x_i,...x_j} is a pair
 * phi = (F,C), where F is a set of temporal assertions, i.e., events and
 * persistence conditions about the state variables x_i,...,x_j, and C
 * is a set of object constraints and temporal constraints."
 * (Automated Planning -- Theory and Practice)
 *
 * \see Timeline
 */
class Chronicle
{
    std::map<StateVariable, Timeline> mTimeline;

public:
    /**
     * Check if chronicle is consistent
     * Iterates over all timelines to check if they are consistent
     */
    bool isConsistent() const;

    /**
     * Check if chronicle supports another chronicle
     */
    bool isSupporting(const Chronicle& other) const;
};

} // end namespace templ
#endif // TEMPL_CHRONICLE_HPP
