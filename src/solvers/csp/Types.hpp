#ifndef TEMPL_SOLVERS_CSP_TYPES_HPP
#define TEMPL_SOLVERS_CSP_TYPES_HPP

#include <vector>
#include <gecode/set.hh>
#include "../../SpaceTime.hpp"

namespace templ {
namespace solvers {
namespace csp {


/// An adjacency list, e.g., which allows to represent a timeline in the CSP
/// Each set represents a node, and each set member a target node
typedef Gecode::SetVarArray AdjacencyList;

/// ListOfAdjecencyLists, e.g., allows to represent a set of timelines in the CSP
typedef std::vector<AdjacencyList> ListOfAdjacencyLists;

/**
 * A conversion class from/to the CSP internal types
 */
class TypeConversion
{
public:
    /**
     * Convert an adjacency list to the SpaceTime timeline representation using
     * the known locations and timepoints
     * \param doThrow set to true to throw when an unassigned view is encountered
     * \return Timeline
     */
    static SpaceTime::Timeline toTimeline(const AdjacencyList& list,
            const std::vector<symbols::constants::Location::Ptr>& locations,
            const std::vector<solvers::temporal::point_algebra::TimePoint::Ptr>& timepoints,
            bool doThrow = true
            );

    /**
     * Convert a list of adjacency lists corresponding to individual roles
     * to the SpaceTime timeline representation using
     * the known locations and timepoints
     *
     * \return Timelines
     */
    static SpaceTime::Timelines toTimelines(const Role::List& roles, const ListOfAdjacencyLists& lists,
            const std::vector<symbols::constants::Location::Ptr>& locations,
            const std::vector<solvers::temporal::point_algebra::TimePoint::Ptr>& timepoints,
            bool doThrow = true
            );
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TYPES_HPP
