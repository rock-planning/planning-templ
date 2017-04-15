#ifndef TEMPL_SOLVERS_TRANSSHIPMENT_TRANSPORT_NETWORK_HPP
#define TEMPL_SOLVERS_TRANSSHIPMENT_TRANSPORT_NETWORK_HPP

#include <templ/Mission.hpp>
#include <templ/SpaceTime.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>

namespace templ {
namespace solvers {
namespace transshipment {

/**
 * Graph-based representation of mobile systems
 */
class TransportNetwork
{
public:
    TransportNetwork(const Mission::Ptr& mission,
        const std::map<Role, csp::RoleTimeline>& timelines);

    const Mission::Ptr& getMission() const { return mpMission; }

    /**
     * Get the graph representation of the transport network, i.e. as a space time network
     */
    SpaceTime::Network& getSpaceTimeNetwork() { return mSpaceTimeNetwork; }
    const std::map<Role, csp::RoleTimeline> getTimeslines() const { return mTimelines; }

protected:
    void initialize();

    void save();

private:
    Mission::Ptr mpMission;
    SpaceTime::Network mSpaceTimeNetwork;
    std::map<Role, csp::RoleTimeline> mTimelines;

};


} // end namespace transsshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TRANSSHIPMENT_TRANSPORT_NETWORK_HPP
