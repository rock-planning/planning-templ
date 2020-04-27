#include "FlowNetwork.hpp"

#include <organization_model/facades/Robot.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <base-logging/Logging.hpp>
#include "../../utils/Logger.hpp"

namespace pa = templ::solvers::temporal::point_algebra;
namespace co = templ::symbols::constants;

using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace transshipment {

FlowNetwork::FlowNetwork(
        const std::map<Role, csp::RoleTimeline>& expandedTimelines,
        const std::map<Role, csp::RoleTimeline>& minRequiredTimelines,
        const symbols::constants::Location::PtrList& locations,
        const temporal::point_algebra::TimePoint::PtrList& sortedTimepoints,
        const organization_model::OrganizationModelAsk& ask,
        const utils::Logger::Ptr& logger
    )
    : mExpandedTimelines(expandedTimelines)
    , mMinRequiredTimelines(minRequiredTimelines)
    , mSpaceTimeNetwork(locations, sortedTimepoints)
    , mMoveToResource()
    , mAsk(ask)
    , mpLogger(logger)
{
    /// register requirements first
    initialize(minRequiredTimelines, true);
    /// register fully routed agent
    initialize(expandedTimelines, false);

    using namespace organization_model;
    mMoveToResource.insert( Resource(vocabulary::OM::resolve("MoveTo")) );
}

void FlowNetwork::save(const std::string& path)
{
    std::string filename = path;
    if(filename.empty())
    {
        if(mpLogger)
        {
            filename = mpLogger->filename("transhipment-flow-network.gexf");
        }
    }
    using namespace graph_analysis::io;

    mSpaceTimeNetwork.save(filename, "gexf");

    LOG_DEBUG_S << "Written transport network to: " << filename;
    LOG_DEBUG_S << "(e.g. view with 'templ-gui " << filename << "'" << ")";

}

void FlowNetwork::initialize(const std::map<Role, csp::RoleTimeline>& timelines, bool updateMinRequirements)
{
    // -----------------------------------------
    // Add capacity-weighted edges to the graph
    // -----------------------------------------
    // Per Role --> add capacities (in terms of capability of carrying an immobile system)
    for(const std::pair<Role, csp::RoleTimeline>& p : timelines)
    {
        // infer connections from timeline
        // sequentially ordered timeline
        // locations and timeline
        // connection from (l0, i0_end) ---> (l1, i1_start)
        //
        const Role& role = p.first;
        const csp::RoleTimeline& roleTimeline = p.second;

        // Check if this item is mobile, i.e. change change the location
        // WARNING: this is domain specific
        // transportCapacity
        using namespace organization_model::facades;
        const Robot& robot = roleTimeline.getRobot();
        if(!robot.isMobile())
        {
            continue;
        }

        uint32_t capacity = robot.getTransportCapacity();
        LOG_DEBUG_S << "Role: " << role.toString() << std::endl
            << "    transport capacity: " << capacity << std::endl;
        LOG_INFO_S << "Process (time-sorted) timeline: " << roleTimeline.toString();

        namespace pa = templ::solvers::temporal::point_algebra;
        SpaceTime::Network::tuple_t::Ptr edgeSourceTuple, edgeTargetTuple;

        RoleInfo::Tag tag = RoleInfo::ASSIGNED;
        for(const SpaceTime::Point& p : roleTimeline.getTimeline())
        {
            SpaceTime::Network::tuple_t::Ptr tuple =
                mSpaceTimeNetwork.tupleByKeys(p.first, p.second);

            if(updateMinRequirements)
            {
                tuple->addRole(role, RoleInfo::REQUIRED);
                continue;
            } else {
                tuple->addRole(role, tag);
            }

            edgeTargetTuple = tuple;
            if(edgeSourceTuple)
            {
                std::vector< RoleInfoWeightedEdge::Ptr > edges = mSpaceTimeNetwork.getGraph()->getEdges<RoleInfoWeightedEdge>(edgeSourceTuple, edgeTargetTuple);
                if(edges.empty())
                {
                    RoleInfoWeightedEdge::Ptr weightedEdge(new RoleInfoWeightedEdge(edgeSourceTuple, edgeTargetTuple, capacity));
                    weightedEdge->addRole(role, RoleInfo::ASSIGNED);
                    mSpaceTimeNetwork.getGraph()->addEdge(weightedEdge);
                } else if(edges.size() > 1)
                {
                    throw
                        std::runtime_error("templ::solvers::transshipment::FlowNetwork: multiple capacity edges detected");
                } else if(edgeSourceTuple->first() != edgeTargetTuple->first())
                {  // one edge -- sum up capacities of mobile systems
                    RoleInfoWeightedEdge::Ptr& existingEdge = edges[0];
                    double existingCapacity = existingEdge->getWeight();

                    if(!existingEdge->hasRole(role, tag))
                    {
                        existingEdge->addRole(role, tag);

                        if(existingCapacity < std::numeric_limits<RoleInfoWeightedEdge::value_t>::max())
                        {
                            capacity += existingCapacity;
                            existingEdge->setWeight(capacity, 0 /*index of 'overall capacity'*/);
                        }
                    } else {
                        throw
                            std::runtime_error("templ::solvers::transshipment::FlowNetwork"
                                    "::initializeExpandedTimelines:"
                                    "Edge has already been assigned for role: " +
                                    role.toString());
                    }
                } else {
                    // local transition edge -- capacity is infinite anyhow
                }
            }
            edgeSourceTuple = tuple;
        }
    }
}

transshipment::Flaw::List FlowNetwork::getInvalidTransitions(double feasibilityCheckTimeoutInMs) const
{
    organization_model::OrganizationModelAsk ask = mAsk;
    transshipment::Flaw::List flaws;
    if(feasibilityCheckTimeoutInMs <= 0)
    {
        // checking disabled
        return flaws;
    }

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
                    << pool.toString(4);
                organization_model::ModelPool::List coalitionStructure =
                    ask.findFeasibleCoalitionStructure(pool, mMoveToResource, feasibilityCheckTimeoutInMs);
                if( coalitionStructure.empty() )
                {
                    LOG_WARN_S << "Infeasible coalition detected for transition"
                        << edge->toString(4)
                        << pool.toString(4)
                        << "timeout: " << feasibilityCheckTimeoutInMs;

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
