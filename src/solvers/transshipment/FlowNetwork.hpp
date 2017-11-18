#ifndef TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP
#define TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP

#include <templ/Mission.hpp>
#include <templ/SpaceTime.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>

namespace templ {
namespace solvers {
namespace transshipment {

/**
 * Graph-based representation of mobile system requirements
 *
 */
class FlowNetwork
{
public:
    /**
     * \param minimalTimelines The set of minimal requirement per role
     * \param expandedTimelines The set of additional requirements for roles
     */
    FlowNetwork(const Mission::Ptr& mission,
        const std::map<Role, csp::RoleTimeline>& minimalTimelines,
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
     * Get the row index of a vertex for supporting a GridLayout
     * \return row index
     */
    size_t getRowIndex(const graph_analysis::Vertex::Ptr& vertex) const;

    /**
     * Get the column index of a vertex for supporting a GridLayout
     * \return column index
     */
    size_t getColumnIndex(const graph_analysis::Vertex::Ptr& vertex) const;

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

    std::map<Role, csp::RoleTimeline> mTimelines;
    SpaceTime::Timelines mExpandedTimelines;
};


} // end namespace transsshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP
