#include "FlowNetwork.hpp"
#include <organization_model/facets/Robot.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <templ/utils/Logger.hpp>
#include <base-logging/Logging.hpp>

namespace pa = templ::solvers::temporal::point_algebra;
namespace co = templ::symbols::constants;

using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace transshipment {

FlowNetwork::FlowNetwork(const Mission::Ptr& mission,
        const std::map<Role, csp::RoleTimeline>& timelines,
        const SpaceTime::Timelines& expandedTimelines)
    : mpMission(mission)
    , mSpaceTimeNetwork(mpMission->getLocations(), mpMission->getTimepoints())
    , mTimelines(timelines)
    , mExpandedTimelines(expandedTimelines)
{
    assert(mpMission);

    if(!mpMission->getTemporalConstraintNetwork()->isConsistent())
    {
        std::string filename = mpMission->getLogger()->filename("transport-network-temporally-constrained-network.dot");
        graph_analysis::io::GraphIO::write(filename, mpMission->getTemporalConstraintNetwork()->getGraph());
        LOG_DEBUG_S << "Written temporal constraint network to: " << filename;
        LOG_DEBUG_S << "(e.g. view with 'xdot " << filename << "'" << ")";

        throw std::runtime_error("templ::solvers::transshipment::FlowNetwork: "
                "cannot create transport network from mission with inconsistent temporal constraint"
                " network");
    }

    initialize();
}

void FlowNetwork::save(const std::string& path)
{
    std::string filename = path;
    if(filename.empty())
    {
        filename = mpMission->getLogger()->filename("transhipment-flow-network.gexf");
    }
    using namespace graph_analysis::io;

    mSpaceTimeNetwork.save(filename, "gexf");

    LOG_DEBUG_S << "Written transport network to: " << filename;
    LOG_DEBUG_S << "(e.g. view with 'templ-gui " << filename << "'" << ")";

}

void FlowNetwork::initialize()
{
    if(mExpandedTimelines.empty())
    {
        initializeMinimalTimelines(false);
    } else {
        initializeMinimalTimelines(true);
        initializeExpandedTimelines();
    }
}

void FlowNetwork::initializeExpandedTimelines()
{
    // -----------------------------------------
    // Add capacity-weighted edges to the graph
    // -----------------------------------------
    // Per Role --> add capacities (in terms of capability of carrying an immobile system)
    SpaceTime::Timelines::const_iterator rit = mExpandedTimelines.begin();
    for(; rit != mExpandedTimelines.end(); ++rit)
    {
        // infer connections from timeline
        // sequentially ordered timeline
        // locations and timeline
        // connection from (l0, i0_end) ---> (l1, i1_start)
        //
        const Role& role = rit->first;
        SpaceTime::Timeline roleTimeline = rit->second;

        // Check if this item is mobile, i.e. change change the location
        // WARNING: this is domain specific based on using the dataProperty payloadTransportCapacity
        //
        organization_model::facets::Robot robot(role.getModel(), mpMission->getOrganizationModelAsk());
        if(!robot.isMobile())
        {
            continue;
        }

        LOG_DEBUG_S << "Add capacity for mobile system: " << role.getModel();
        uint32_t capacity = robot.getPayloadTransportCapacity();
        LOG_DEBUG_S << "Role: " << role.toString() << std::endl
            << "    transport capacity: " << capacity << std::endl;

        namespace pa = templ::solvers::temporal::point_algebra;
        pa::TimePoint::Ptr prevIntervalEnd;
        co::Location::Ptr prevLocation;
        SpaceTime::Network::tuple_t::Ptr startTuple, endTuple;

        // Iterator over all role-specific timelines and annotate vertices and
        // edges with the information of the role that is assigned to it
        SpaceTime::Timeline::const_iterator cit = roleTimeline.begin();
        RoleInfo::Tag tag = RoleInfo::AVAILABLE;
        for(; cit != roleTimeline.end(); ++cit)
        {
            const SpaceTime::Point& spaceTimePoint = *cit;
            symbols::constants::Location::Ptr location = spaceTimePoint.first;
            solvers::temporal::point_algebra::TimePoint::Ptr timepoint = spaceTimePoint.second;

            // create tuple if it does not exist?
            endTuple = mSpaceTimeNetwork.tupleByKeys(location, timepoint);
            endTuple->addRole(role, tag);

            if(prevIntervalEnd)
            {
                startTuple = mSpaceTimeNetwork.tupleByKeys(prevLocation, prevIntervalEnd);
                startTuple->addRole(role, tag);

                std::vector< RoleInfoWeightedEdge::Ptr > edges = mSpaceTimeNetwork.getGraph()->getEdges<RoleInfoWeightedEdge>(startTuple, endTuple);
                if(edges.empty())
                {
                    RoleInfoWeightedEdge::Ptr weightedEdge(new RoleInfoWeightedEdge(startTuple, endTuple, capacity));
                    weightedEdge->addRole(role, tag);
                    mSpaceTimeNetwork.getGraph()->addEdge(weightedEdge);
                } else if(edges.size() > 1)
                {
                    throw std::runtime_error("templ::solvers::transshipment::FlowNetwork: multiple capacity edges detected");
                } else { // one edge -- sum up capacities of mobile systems
                    RoleInfoWeightedEdge::Ptr& existingEdge = edges[0];
                    double existingCapacity = existingEdge->getWeight();
                    existingEdge->addRole(role, tag);

                    if(existingCapacity < std::numeric_limits<RoleInfoWeightedEdge::value_t>::max())
                    {
                        capacity += existingCapacity;
                        existingEdge->setWeight(capacity, 0 /*index of 'overall capacity'*/);
                    }
                }
            } else {
                tag = RoleInfo::ASSIGNED;
            }

            prevIntervalEnd = timepoint;
            prevLocation = location;
        }
    }
}

void FlowNetwork::initializeMinimalTimelines(bool updateRolesOnly)
{
    // -----------------------------------------
    // Add capacity-weighted edges to the graph
    // -----------------------------------------
    // Per Role --> add capacities (in terms of capability of carrying an immobile system)
    std::map<Role, csp::RoleTimeline>::const_iterator rit = mTimelines.begin();
    for(; rit != mTimelines.end(); ++rit)
    {
        // infer connections from timeline
        // sequentially ordered timeline
        // locations and timeline
        // connection from (l0, i0_end) ---> (l1, i1_start)
        //
        const Role& role = rit->first;
        csp::RoleTimeline roleTimeline = rit->second;

        // Check if this item is mobile, i.e. change change the location
        // WARNING: this is domain specific
        // transportCapacity
        organization_model::facets::Robot robot(role.getModel(), mpMission->getOrganizationModelAsk());
        if(!robot.isMobile())
        {
            continue;
        }

        LOG_DEBUG_S << "Add capacity for mobile system: " << role.getModel();
        uint32_t capacity = robot.getPayloadTransportCapacity();
        LOG_DEBUG_S << "Role: " << role.toString() << std::endl
            << "    transport capacity: " << capacity << std::endl;

        namespace pa = templ::solvers::temporal::point_algebra;
        pa::TimePoint::Ptr prevIntervalEnd;
        co::Location::Ptr prevLocation;
        SpaceTime::Network::tuple_t::Ptr startTuple, endTuple;

        LOG_INFO_S << "Process (time-sorted) timeline: " << roleTimeline.toString();
        const std::vector<csp::FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
        std::vector<csp::FluentTimeResource>::const_iterator fit = ftrs.begin();
        RoleInfo::Tag tag = RoleInfo::AVAILABLE;
        for(; fit != ftrs.end(); ++fit)
        {
            symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
            solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

            LOG_WARN_S << "Location: " << location->toString() << " -- interval: " << interval.toString();

            // create tuple if it does not exist?
            endTuple = mSpaceTimeNetwork.tupleByKeys(location, interval.getFrom());
            endTuple->addRole(role, tag);

            // Find start node: Tuple of location and interval.getFrom()
            if(prevIntervalEnd)
            {
                startTuple = mSpaceTimeNetwork.tupleByKeys(prevLocation, prevIntervalEnd);
                startTuple->addRole(role, tag);

                if(updateRolesOnly)
                {
                    continue;
                }

                std::vector< RoleInfoWeightedEdge::Ptr > edges = mSpaceTimeNetwork.getGraph()->getEdges<RoleInfoWeightedEdge>(startTuple, endTuple);
                if(edges.empty())
                {
                    RoleInfoWeightedEdge::Ptr weightedEdge(new RoleInfoWeightedEdge(startTuple, endTuple, capacity));
                    mSpaceTimeNetwork.getGraph()->addEdge(weightedEdge);
                } else if(edges.size() > 1)
                {
                    throw std::runtime_error("MissionPlanner: multiple capacity edges detected");
                } else { // one edge -- sum up capacities of mobile systems
                    RoleInfoWeightedEdge::Ptr& existingEdge = edges[0];
                    double existingCapacity = existingEdge->getWeight();
                    if(existingCapacity < std::numeric_limits<RoleInfoWeightedEdge::value_t>::max())
                    {
                        capacity += existingCapacity;
                        existingEdge->setWeight(capacity, 0 /*index of 'overall capacity'*/);
                    }
                }
            } else {
                tag = RoleInfo::REQUIRED;
            }

            prevIntervalEnd = interval.getTo();
            prevLocation = location;
        }
    }
}

} // end namespace transsshipment
} // end namespace solvers
} // end namespace templ
