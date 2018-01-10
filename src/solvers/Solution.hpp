#ifndef TEMPL_SOLVERS_SOLUTION_HPP
#define TEMPL_SOLVERS_SOLUTION_HPP

#include <vector>
#include "../SpaceTime.hpp"

namespace templ {
namespace solvers {

/**
 * \class Solution
 * \brief A complete solution allow to represent the final assignment of robots to requirements, the
 * required transitions and the quantification of time windows
 *
 * \details A partial solution contains only the final assignment of robots,
 * It should also allow the extraction of reconfiguration elements
 * (pickup/delivery)
 *
 * [location/time: role a_0, ... role a_n]
 * \see SolutionAnalysis
 */
class Solution
{
public:
    typedef std::vector<Solution> List;

    /// The space time network contains the / the (min cost flow) solution.
    /// For the representation of
    ///     - vertices: SpaceTime::RoleInfoSpaceTimeTuple
    ///     - edges:    RoleInfoWeightedEdge
    SpaceTime::Network mSpaceTimeNetwork;

    // Each vertex: requires a description on how to reconfigure:
    // start-reconf + end-reconf (split/join -- split/join)
    // start-coalition/agent --> core-coalition/agent --> end-coalition/agent
    // input: all incoming agent coalitions (in edges), all outgoing agent
    // coalitions (out edges)
    // todo:
    //     - cost function/heuristic: estimate cost and time for transition (objective --
    // efficiency, effective, robust transition) --
    //     - also allow to check whether transfer is feasible at all (manipulation capability), though we assume
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_SOLUTION_HPP
