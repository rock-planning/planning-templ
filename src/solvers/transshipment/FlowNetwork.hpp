#ifndef TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP
#define TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP

#include "../../Mission.hpp"
#include "../../SpaceTime.hpp"
#include "../csp/RoleTimeline.hpp"
#include "Flaw.hpp"

namespace templ {
namespace solvers {
namespace transshipment {

/**
 * Graph-based representation of mobile system requirements
 * The requirements are given either as part of a minimal/sparse timeline, i.e.
 * the minimum requirements to be fulfilled or a fully expanded timeline
 *
 * The demand corresponding to the immobile systems will be set through
 * MinCostFlow
 * \see MinCostFlow
 *
 * \todo improve FlowNetwork representation, the reason for this split is
 * the support of diverge code paths
 */
class FlowNetwork
{
public:
    /**
     * Default constructor of the flow network
     * \param timelines Timeline that represent the fully expanded timelines * (after csp timeline branching)
     * \param minRequiredTimeline represents the timelines that are based on the
     * minimum requirements
     * \param locations set of locations
     * \param sortedTimepoints set of time sorted timepoints
     * \param ask organization model ask object
     * \param logger logger instance
     */
    FlowNetwork(const std::map<Role, csp::RoleTimeline>& fullyExpandedTimelines,
        const std::map<Role, csp::RoleTimeline>& minRequiredTimelines,
        const symbols::constants::Location::PtrList& locations,
        const temporal::point_algebra::TimePoint::PtrList& sortedTimepoints,
        const moreorg::OrganizationModelAsk& ask,
        const utils::Logger::Ptr& logger = utils::Logger::Ptr());

    /**
     * Get the graph representation of the transport network, i.e. as a space time network
     *
     * The transport network will contain RoleInfo tuples which are augmented
     * with the roles that are assigned to it under the key 'assigned'
     */
    const SpaceTime::Network& getSpaceTimeNetwork() const { return mSpaceTimeNetwork; }

    /**
     * Get the current set of timelines
     */
    const std::map<Role, csp::RoleTimeline> getExpandedTimeslines() const { return mExpandedTimelines; }

    /**
     * Get the current set of timelines
     */
    const std::map<Role, csp::RoleTimeline> getMinRequiredTimeslines() const { return mMinRequiredTimelines; }

    void save(const std::string& filename = "");

    /**
     * Check feasibility of a transition for the given
     * agent
     * \param feasibilityCheckTimeoutInMs Checking the feasibility might
     * encounter infeasible solutions which will be however be explored
     * exhaustively until the given timeout is reached
     */
    transshipment::Flaw::List getInvalidTransitions(double
            feasibilityCheckTimeoutInMs = 1000) const;

protected:
    /**
     * check if expanded or minimal timelines shall be used for initialization
     * and call appropriate initialization function
     */
    void initialize(const std::map<Role, csp::RoleTimeline>& timeline,
            bool updateMinRequirements);

    /**
     * Initialize mobile agent timelines
     */
    void initializeMobileAgentTimelines(bool updateRolesOnly = false);

    /**
     * Initialize network (from a
     */
    void initializeExpandedTimelines();

private:
    std::map<Role, csp::RoleTimeline> mExpandedTimelines;
    std::map<Role, csp::RoleTimeline> mMinRequiredTimelines;

    SpaceTime::Network mSpaceTimeNetwork;
    /// Allow to check valid transitions
    moreorg::Resource::Set mMoveToResource;

    moreorg::OrganizationModelAsk mAsk;
    utils::Logger::Ptr mpLogger;
};


} // end namespace transsshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP
