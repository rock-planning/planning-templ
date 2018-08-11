#include "FlowNetwork.hpp"

#include <organization_model/facades/Robot.hpp>
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
        const temporal::point_algebra::TimePoint::PtrList& sortedTimepoints,
        const std::map<Role, csp::RoleTimeline>& timelines,
        const SpaceTime::Timelines& expandedTimelines)
    : mpMission(mission)
    , mSpaceTimeNetwork(mission->getLocations(), sortedTimepoints)
    , mAsk(mission->getOrganizationModelAsk())
    , mTimelines(timelines)
    , mExpandedTimelines(expandedTimelines)
    , mMoveToResource()
{
    assert(mpMission);
    initialize();

    using namespace organization_model;
    mMoveToResource.insert( Resource(vocabulary::OM::resolve("MoveTo")) );
}

FlowNetwork::FlowNetwork(const organization_model::OrganizationModelAsk& ask,
        const symbols::constants::Location::PtrList locations,
        const temporal::point_algebra::TimePoint::PtrList& sortedTimepoints,
        const SpaceTime::Timelines& expandedTimelines)
    : mSpaceTimeNetwork(locations, sortedTimepoints)
    , mAsk(ask)
    , mTimelines()
    , mExpandedTimelines(expandedTimelines)
    , mMoveToResource()
{
    initialize();

    using namespace organization_model;
    mMoveToResource.insert( Resource(vocabulary::OM::resolve("MoveTo")) );
}

void FlowNetwork::save(const std::string& path)
{
    std::string filename = path;
    if(filename.empty())
    {
        if(mpMission)
        {
            filename = mpMission->getLogger()->filename("transhipment-flow-network.gexf");
        } else {
            filename = "/tmp/transhipment-flow-network.gexf";
        }
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

        // Check if this item is mobile, i.e. change the location
        // WARNING: this is currently domain specific based on using the dataProperty payloadTransportCapacity
        //
        organization_model::facades::Robot robot = organization_model::facades::Robot::getInstance(role.getModel(), mAsk);
        if(!robot.isMobile())
        {
            continue;
        }

        /// MOBILE SYSTEMS ONLY
        uint32_t capacity = robot.getTransportCapacity();

        namespace pa = templ::solvers::temporal::point_algebra;
        pa::TimePoint::Ptr prevIntervalEnd;
        co::Location::Ptr prevLocation;
        SpaceTime::Network::tuple_t::Ptr startTuple, endTuple;

        // Iterate over all role-specific timelines and annotate vertices and
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
                    if(!existingEdge->hasRole(role, tag))
                    {
                        existingEdge->addRole(role, tag);

                        if(existingCapacity < std::numeric_limits<RoleInfoWeightedEdge::value_t>::max())
                        {
                            uint32_t newCapacity = existingCapacity + capacity;
                            existingEdge->setWeight(newCapacity, 0 /*index of 'overall capacity'*/);
                        }
                    } else {
                        throw std::runtime_error("templ::solvers::transshipment::FlowNetwork::initializeExpandedTimelines: Edge has already been assigned for role: "
                                + role.toString());
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
        organization_model::facades::Robot robot = organization_model::facades::Robot::getInstance(role.getModel(), mAsk);
        if(!robot.isMobile())
        {
            continue;
        }

        LOG_DEBUG_S << "Add capacity for mobile system: " << role.getModel();
        uint32_t capacity = robot.getTransportCapacity();
        LOG_DEBUG_S << "Role: " << role.toString() << std::endl
            << "    transport capacity: " << capacity << std::endl;

        namespace pa = templ::solvers::temporal::point_algebra;
        SpaceTime::Network::tuple_t::Ptr edgeSourceTuple, edgeTargetTuple;

        LOG_INFO_S << "Process (time-sorted) timeline: " << roleTimeline.toString();
        const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
        std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
        RoleInfo::Tag tag = RoleInfo::REQUIRED;
        for(; fit != ftrs.end(); ++fit)
        {
            const FluentTimeResource& ftr = *fit;
            symbols::constants::Location::Ptr location = ftr.getLocation();
            solvers::temporal::Interval interval = ftr.getInterval();

            // Get all involved tuples
            SpaceTime::Network::tuple_t::PtrList tuples = mSpaceTimeNetwork.getTuples(interval.getFrom(), interval.getTo(), location);
            for(const SpaceTime::Network::tuple_t::Ptr& tuple : tuples)
            {
                tuple->addRole(role, tag);
            }

            if(updateRolesOnly)
            {
                continue;
            }

            // create tuple if it does not exist?
            edgeTargetTuple = tuples.front();

            // Find start node: Tuple of location and interval.getFrom()
            // This is where the agent is available
            if(edgeSourceTuple)
            {
                std::vector< RoleInfoWeightedEdge::Ptr > edges = mSpaceTimeNetwork.getGraph()->getEdges<RoleInfoWeightedEdge>(edgeSourceTuple, edgeTargetTuple);
                if(edges.empty())
                {
                    RoleInfoWeightedEdge::Ptr weightedEdge(new RoleInfoWeightedEdge(edgeSourceTuple, edgeTargetTuple, capacity));
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
            }
            edgeSourceTuple = tuples.back();
        }
    }
}

transshipment::Flaw::List FlowNetwork::getInvalidTransitions(double feasibilityCheckTimeoutInMs) const
{
    organization_model::OrganizationModelAsk ask = mAsk;

    transshipment::Flaw::List flaws;

    EdgeIterator::Ptr edgeIt =  mSpaceTimeNetwork.getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {
        Edge::Ptr edge = edgeIt->current();
        RoleInfoWeightedEdge::Ptr roleInfo = dynamic_pointer_cast<RoleInfoWeightedEdge>(edge);
        if(roleInfo)
        {
            if(roleInfo->getWeight() < std::numeric_limits<uint32_t>::max())
            {
                Role::Set roles = roleInfo->getRoles({ RoleInfo::ASSIGNED });
                organization_model::ModelPool pool = roleInfo->getModelPool( { RoleInfo::ASSIGNED } );

                LOG_INFO_S << "Checking for infeasible coalition on transition:"
                    << edge->toString(4);
                organization_model::ModelPool::List coalitionStructure =
                    ask.findFeasibleCoalitionStructure(pool, mMoveToResource, feasibilityCheckTimeoutInMs);
                if( coalitionStructure.empty() )
                {
                    LOG_WARN_S << "Infeasible coalition detected for transition"
                        << edge->toString(4);
                    using namespace graph_analysis::algorithms;
                    ConstraintViolation v(MultiCommodityEdge::Ptr(),
                            std::set<uint32_t>(), 0, 0,0, ConstraintViolation::TotalTransFlow);

                    SpaceTime::Network::tuple_t::Ptr from = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(roleInfo->getSourceVertex());
                    SpaceTime::Network::tuple_t::Ptr to = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(roleInfo->getTargetVertex());

                    Flaw flaw(v, Role::List(roles.begin(), roles.end()),
                            from->getPair(), to->getPair());
                    flaws.push_back(flaw);

                    for(const Role& r : roles)
                    {
                        roleInfo->addRole(r, RoleInfo::INFEASIBLE);
                    }
                } else {
                    LOG_INFO_S << "Feasible coalition detected for transition"
                        << edge->toString(4);
                }
            }
        }
    }

    return flaws;
}

} // end namespace transsshipment
} // end namespace solvers
} // end namespace templ
