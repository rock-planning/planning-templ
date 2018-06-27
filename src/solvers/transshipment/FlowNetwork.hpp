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
     * \param mission Mission to which this flow network is related
     * \param sortedTimepoints List of sorted timepoints (which allows to create
     * the temporally expanded network)
     * \param minimalTimelines The set of minimal requirement per role
     * \param expandedTimelines The set of additional requirements for roles
     */
    FlowNetwork(const Mission::Ptr& mission,
        const temporal::point_algebra::TimePoint::PtrList& sortedTimepoints,
        const std::map<Role, csp::RoleTimeline>& minimalTimelines,
        const SpaceTime::Timelines& expandedTimelines = SpaceTime::Timelines());

    /**
     * \param ask
     * \param sortedTimepoints List of sorted timepoints (which allows to create
     * the temporally expanded network)
     * \param minimalTimelines The set of minimal requirement per role
     * \param expandedTimelines The set of additional requirements for roles
     */
    FlowNetwork(const organization_model::OrganizationModelAsk& ask,
        const symbols::constants::Location::PtrList locations,
        const temporal::point_algebra::TimePoint::PtrList& sortedTimepoints,
        const SpaceTime::Timelines& expandedTimelines = SpaceTime::Timelines());

    const Mission::Ptr& getMission() const { return mpMission; }

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
    const std::map<Role, csp::RoleTimeline> getTimeslines() const { return mTimelines; }

    /**
     *
     */
    const SpaceTime::Timelines getExpandedTimelines() const { return mExpandedTimelines; }

    void save(const std::string& filename = "");

    /**
     * Check feasibility of a transition for the given
     * agent
     */
    transshipment::Flaw::List getInvalidTransitions(double feasibilityCheckTimeoutInS = 1) const;

protected:
    /**
     * check if expanded or minimal timelines shall be used for initialization
     * and call appropriate initialization function
     */
    void initialize();

    /**
     * Initialize network (from a vector of FluentTimeResource)
     *
     * This initialize the minimal required elements of a timeline
     */
    void initializeMinimalTimelines(bool updateRolesOnly = false);

    /**
     * Initialize network (from a
     */
    void initializeExpandedTimelines();

private:
    Mission::Ptr mpMission;
    SpaceTime::Network mSpaceTimeNetwork;
    organization_model::OrganizationModelAsk mAsk;

    std::map<Role, csp::RoleTimeline> mTimelines;
    SpaceTime::Timelines mExpandedTimelines;

    /// Allow to check valid transitions
    organization_model::Resource::Set mMoveToResource;
};


} // end namespace transsshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP
