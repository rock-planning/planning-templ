#ifndef TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP
#define TEMPL_SOLVERS_TRANSSHIPMENT_FLOW_NETWORK_HPP

#include <templ/Mission.hpp>
#include <templ/SpaceTime.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>

namespace templ {
namespace solvers {
namespace transshipment {

/**
 * Graph-based representation of mobile systems
 */
class FlowNetwork
{
public:
    /**
     * TODO: currently using timelines and expandedTimelines
     * was intended to deal with different timeline definition
     */
    FlowNetwork(const Mission::Ptr& mission,
        const std::map<Role, csp::RoleTimeline>& timelines,
        const SpaceTime::Timelines& expandedTimelines = SpaceTime::Timelines());

    const Mission::Ptr& getMission() const { return mpMission; }

    /**
     * Get the graph representation of the transport network, i.e. as a space time network
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
    void initialize();
    void initializeMinimalTimelines();
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
