#ifndef TEMPL_SOLVERS_SOLUTION_HPP
#define TEMPL_SOLVERS_SOLUTION_HPP

#include <vector>
#include "../SpaceTime.hpp"

namespace templ {
    class Mission;
}

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

    Solution();

    Solution(const SpaceTime::Network& network,
            const moreorg::OrganizationModel::Ptr& om);

    const SpaceTime::Network& getSpaceTimeNetwork() { return mSpaceTimeNetwork; }

    void setSpaceTimeNetwork(const SpaceTime::Network& network) { mSpaceTimeNetwork = network; }

    /**
     * return the underlying graph of the space time network
     */
    graph_analysis::BaseGraph::Ptr getGraph() { return mSpaceTimeNetwork.getGraph(); }

    /**
     * Get list of temporally ordered timepoints
     */
    const temporal::point_algebra::TimePoint::PtrList& getTimepoints() const { return mSpaceTimeNetwork.getTimepoints(); }

    /**
     * Get list of locations
     */
    const symbols::constants::Location::PtrList& getLocations() const { return mSpaceTimeNetwork.getValues(); }

    /**
     *  Get all Agent Roles which are involved in this solution
     */
    Role::Set getAgentRoles() const;

    /**
     * Get the start for a particular role
     */
    const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& getStart(const Role& role) const;

    /**
     * Get the route for this particular role
     * \return List of SpaceTime::RoleInfoSpaceTimeTuple::Ptr
     */
    SpaceTime::Route getRoute(const Role& role) const;

    /**
     * Allow to change a solution
     * \return SpaceTime::RoleInfoSpaceTimeTuple::Ptr where the role has been removed
     */
    SpaceTime::RoleInfoSpaceTimeTuple::Ptr removeRole(const Role& role,
            const temporal::point_algebra::TimePoint::Ptr& timepoint,
            const symbols::constants::Location::Ptr& location = symbols::constants::Location::Ptr(),
            RoleInfo::Tag tag = RoleInfo::ASSIGNED);

    /**
     * Add a role to to a space time tuple
     * \return the updated role info tuple
     */
    const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& addRole(const Role& role,
            const temporal::point_algebra::TimePoint::Ptr& timepoint,
            const symbols::constants::Location::Ptr& location,
            RoleInfo::Tag tag = RoleInfo::AVAILABLE);

    SpaceTime::RoleInfoSpaceTimeTuple::Ptr addRole(const Role& role,
            const temporal::point_algebra::TimePoint::Ptr& from,
            const temporal::point_algebra::TimePoint::Ptr& to,
            const symbols::constants::Location::Ptr& location,
            RoleInfo::Tag tag = RoleInfo::AVAILABLE);

    /**
     * Retrieve the edge between two RoleIntoTuple::Ptr
     */
    RoleInfoWeightedEdge::PtrList getEdges(const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& from,
            const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& to);

    /**
     * Save the current solution
     */
    void save(const std::string& filename) const;

    /**
     * Convert solution to mission without considering new mission
     */
    shared_ptr<Mission> toMission(const moreorg::OrganizationModel::Ptr& om,
            const std::string& name = "") const;

    /**
     * Convert solution to mission while augementing existing mission
     */
    shared_ptr<Mission> toMission(const shared_ptr<Mission>& existingMission) const;

protected:
    /// The space time network contains the / the (min cost flow) solution.
    /// For the representation of
    ///     - vertices: SpaceTime::RoleInfoSpaceTimeTuple
    ///     - edges:    RoleInfoWeightedEdge
    SpaceTime::Network mSpaceTimeNetwork;

    moreorg::OrganizationModelAsk mOrganizationModelAsk;

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
