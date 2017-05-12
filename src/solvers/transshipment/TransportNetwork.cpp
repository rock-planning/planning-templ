#include "TransportNetwork.hpp"
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

TransportNetwork::TransportNetwork(const Mission::Ptr& mission,
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

        throw std::runtime_error("templ::solvers::transshipment::TransportNetwork: "
                "cannot create transport network from mission with inconsistent temporal constraint"
                " network");
    }

    initialize();
}

void TransportNetwork::save()
{
    std::string filename = mpMission->getLogger()->filename("transport-network.dot");
    graph_analysis::io::GraphIO::write(filename, mSpaceTimeNetwork.getGraph());
    LOG_DEBUG_S << "Written transport network to: " << filename;
    LOG_DEBUG_S << "(e.g. view with 'xdot " << filename << "'" << ")";
}

void TransportNetwork::initialize()
{
    if(mExpandedTimelines.empty())
    {
        initializeMinimalTimelines();
    } else {
        initializeExpandedTimelines();
    }
}

void TransportNetwork::initializeExpandedTimelines()
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

        SpaceTime::Timeline::const_iterator cit = roleTimeline.begin();
        for(; cit != roleTimeline.end(); ++cit)
        {
            const SpaceTime::Point& spaceTimePoint = *cit;
            symbols::constants::Location::Ptr location = spaceTimePoint.first;
            solvers::temporal::point_algebra::TimePoint::Ptr timepoint = spaceTimePoint.second;

            // create tuple if it does not exist?
            endTuple = mSpaceTimeNetwork.tupleByKeys(location, timepoint);
            endTuple->addRole(role);

            if(prevIntervalEnd)
            {
                startTuple = mSpaceTimeNetwork.tupleByKeys(prevLocation, prevIntervalEnd);
                startTuple->addRole(role);

                std::vector< WeightedEdge::Ptr > edges = mSpaceTimeNetwork.getGraph()->getEdges<WeightedEdge>(startTuple, endTuple);
                if(edges.empty())
                {
                    WeightedEdge::Ptr weightedEdge(new WeightedEdge(startTuple, endTuple, capacity));
                    mSpaceTimeNetwork.getGraph()->addEdge(weightedEdge);
                } else if(edges.size() > 1)
                {
                    throw std::runtime_error("MissionPlanner: multiple capacity edges detected");
                } else { // one edge -- sum up capacities of mobile systems
                    WeightedEdge::Ptr& existingEdge = edges[0];
                    double existingCapacity = existingEdge->getWeight();
                    if(existingCapacity < std::numeric_limits<WeightedEdge::value_t>::max())
                    {
                        capacity += existingCapacity;
                        existingEdge->setWeight(capacity, 0 /*index of 'overall capacity'*/);
                    }
                }
            }

            prevIntervalEnd = timepoint;
            prevLocation = location;
        }
    }
}

void TransportNetwork::initializeMinimalTimelines()
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
        for(; fit != ftrs.end(); ++fit)
        {
            symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
            solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

            LOG_WARN_S << "Location: " << location->toString() << " -- interval: " << interval.toString();

            // create tuple if it does not exist?
            endTuple = mSpaceTimeNetwork.tupleByKeys(location, interval.getFrom());
            endTuple->addRole(role);

            // Find start node: Tuple of location and interval.getFrom()
            if(prevIntervalEnd)
            {
                startTuple = mSpaceTimeNetwork.tupleByKeys(prevLocation, prevIntervalEnd);
                startTuple->addRole(role);

                std::vector< WeightedEdge::Ptr > edges = mSpaceTimeNetwork.getGraph()->getEdges<WeightedEdge>(startTuple, endTuple);
                if(edges.empty())
                {
                    WeightedEdge::Ptr weightedEdge(new WeightedEdge(startTuple, endTuple, capacity));
                    mSpaceTimeNetwork.getGraph()->addEdge(weightedEdge);
                } else if(edges.size() > 1)
                {
                    throw std::runtime_error("MissionPlanner: multiple capacity edges detected");
                } else { // one edge -- sum up capacities of mobile systems
                    WeightedEdge::Ptr& existingEdge = edges[0];
                    double existingCapacity = existingEdge->getWeight();
                    if(existingCapacity < std::numeric_limits<WeightedEdge::value_t>::max())
                    {
                        capacity += existingCapacity;
                        existingEdge->setWeight(capacity, 0 /*index of 'overall capacity'*/);
                    }
                }
            }

            prevIntervalEnd = interval.getTo();
            prevLocation = location;
        }
    }
}

} // end namespace transsshipment
} // end namespace solvers
} // end namespace templ
