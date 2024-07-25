#include "MinCostFlow.hpp"
#include <base-logging/Logging.hpp>
#include <graph_analysis/BipartiteGraph.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <moreorg/facades/Robot.hpp>
#include <graph_analysis/algorithms/ConstraintViolation.hpp>

#include "../FluentTimeResource.hpp"
#include "../../utils/Logger.hpp"

using namespace graph_analysis;
using namespace graph_analysis::algorithms;
using namespace templ::solvers::temporal;

namespace templ {
namespace solvers {
namespace transshipment {

MinCostFlow::MinCostFlow(
        const std::map<Role, csp::RoleTimeline>& expandedTimelines,
        const std::map<Role, csp::RoleTimeline>& minRequiredTimelines,
        const symbols::constants::Location::PtrList& locations,
        const temporal::point_algebra::TimePoint::PtrList& sortedTimepoints,
        const moreorg::OrganizationModelAsk& ask,
        const utils::Logger::Ptr& logger,
        graph_analysis::algorithms::LPSolver::Type solverType,
        double feasibilityTimeoutInMs)
    : mExpandedTimelines(expandedTimelines)
    , mMinRequiredTimelines(minRequiredTimelines)
    , mSortedTimepoints(sortedTimepoints)
    , mAsk(ask)
    , mpLogger(logger)
    , mFlowNetwork(expandedTimelines,
            minRequiredTimelines,
            locations,
            sortedTimepoints,
            mAsk,
            mpLogger)
    , mSpaceTimeNetwork(mFlowNetwork.getSpaceTimeNetwork())
    , mSolverType(solverType)
    , mFeasibilityTimeoutInMs(feasibilityTimeoutInMs)
{
    // Create virtual start and end depot vertices and connect them with the
    // current start and end vertices
    SpaceTime::injectVirtualStartAndEnd(mSpaceTimeNetwork);

    for(const std::pair<const Role, csp::RoleTimeline>& p : mExpandedTimelines)
    {
        const Role& role = p.first;
        const csp::RoleTimeline& timeline = p.second;
        if(!timeline.getRobot().isMobile()) // only immobile systems are relevant
        {
            // Allow to later map roles back from index
            mCommoditiesRoles.push_back(role);
        }
    }

    for(const std::pair<const Role, csp::RoleTimeline>& p : mMinRequiredTimelines)
    {
        const Role& role = p.first;
        if(p.second.getTimeline().empty())
        {
            throw std::invalid_argument("templ::solvers::transshipment::"
                    "MinCostFlow "
                    "timeline without minimum requirements available for role '" +
                    role.toString() + "' at least start must be provided");
        }
    }

    if(mCommoditiesRoles.empty())
    {
        LOG_WARN_S << "No immobile systems, thus no commodities to be routed";
    }
    mInitialSetupCost = std::vector<double>(mCommoditiesRoles.size(), 1.0);
}

BaseGraph::Ptr MinCostFlow::createFlowGraph(uint32_t commodities)
{
    BaseGraph::Ptr flowGraph = BaseGraph::getInstance();

    // Create the vertices of the flow, that can be mapped back to the space time
    // network using the bipartiteGraph structure
    VertexIterator::Ptr vertexIt = mSpaceTimeNetwork.getGraph()->getVertexIterator();
    while(vertexIt->next())
    {
        MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex = make_shared<MultiCommodityMinCostFlow::vertex_t>(commodities);
        mBipartiteGraph.linkVertices(multicommodityVertex, vertexIt->current());
        multicommodityVertex->setLabel(vertexIt->current()->toString());
    }

    // Create the edges, i.e. the 'transport-links'
    // Iterator of the existing edges of the transport network
    // and set the commodities
    EdgeIterator::Ptr edgeIt = mSpaceTimeNetwork.getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {
        WeightedEdge::Ptr edge = dynamic_pointer_cast<WeightedEdge>(edgeIt->current());

        Vertex::Ptr source = edge->getSourceVertex();
        Vertex::Ptr target = edge->getTargetVertex();

        bool isHorizonStart = (source == SpaceTime::getHorizonStartTuple());
        bool isHorizonEnd = (target == SpaceTime::getHorizonEndTuple());

        // Create an edge in the multicommodity representation that correspond to the space time graph
        MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge = make_shared<MultiCommodityMinCostFlow::edge_t>(commodities);
        multicommodityEdge->setSourceVertex( mBipartiteGraph.getUniquePartner(source) );
        multicommodityEdge->setTargetVertex( mBipartiteGraph.getUniquePartner(target) );

        double weight = edge->getWeight();
        uint32_t bound = 0;
        if(weight == std::numeric_limits<double>::max())
        {
            bound = std::numeric_limits<uint32_t>::max();
        } else {
            bound = static_cast<uint32_t>(weight);
        }

        // Upper bound is the maximum edge capacity for a commodity
        multicommodityEdge->setCapacityUpperBound(bound);

        for(size_t i = 0; i < commodities; ++i)
        {
            if(isHorizonStart)
            {
                multicommodityEdge->setCommodityCost(i, std::numeric_limits<uint32_t>::max());
                // disable to selective enable through setDepotRestrictions
                multicommodityEdge->setCommodityCapacityUpperBound(i, 0);
            }
            if(isHorizonEnd)
            {
                multicommodityEdge->setCommodityCost(i, 0);
            }

            multicommodityEdge->setCommodityCapacityUpperBound(i, bound);
        }
        flowGraph->addEdge(multicommodityEdge);
    }

    return flowGraph;
}

void MinCostFlow::setCommoditySupplyAndDemand()
{
    for(const std::pair<const Role, csp::RoleTimeline>& p : mMinRequiredTimelines)
    {
        const Role& role = p.first;
        const csp::RoleTimeline& roleTimeline = p.second;
        const SpaceTime::Timeline& timeline = roleTimeline.getTimeline();

        // The list of commodities roles is initialized in the constructor and
        // contains the list of immobile units
        std::vector<Role>::const_iterator cit = std::find(mCommoditiesRoles.begin(), mCommoditiesRoles.end(), role);
        if(cit != mCommoditiesRoles.end())
        {
            moreorg::facades::Robot robot = role.getFacade(mAsk);
            uint32_t transportDemand = robot.getTransportDemand();
            if(transportDemand == 0)
            {
                throw
                    std::invalid_argument("templ::solvers::transshipment::"
                            "MinCostFlow::setCommoditySupplyAndDemand: "
                            "transportDemand for model '" +
                            role.getModel().toString() + "' is 0 -- pls"
                            " update the organization_model");
            }

            size_t commodityId = cit - mCommoditiesRoles.begin();
            setSupplyDemand(SpaceTime::getHorizonStartTuple(),commodityId, role, transportDemand);
            setSupplyDemand(SpaceTime::getHorizonEndTuple(),commodityId, role, -transportDemand);

            for(size_t i = 0; i < timeline.size(); ++i)
            {
                // todo update constraint for all that are assigned inbetween
                SpaceTime::Network::tuple_t::Ptr currentTuple = mSpaceTimeNetwork.tupleByKeys(timeline[i].first, timeline[i].second);
                setMinTransFlow(currentTuple, commodityId, role, transportDemand);
            }
        }
    } // end for role timelines
}

void MinCostFlow::setDepotRestrictions(const graph_analysis::BaseGraph::Ptr& flowGraph,
        uint32_t commodities)
{
    SpaceTime::Network::tuple_t::Ptr startDepot = SpaceTime::getHorizonStartTuple();

    EdgeIterator::Ptr edgeIt = mSpaceTimeNetwork.getGraph()->getOutEdgeIterator(startDepot);
    while(edgeIt->next())
    {
        Edge::Ptr edge = edgeIt->current();

        MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge = getPartner(flowGraph, edge);
        MultiCommodityMinCostFlow::vertex_t::Ptr source = dynamic_pointer_cast<MultiCommodityMinCostFlow::vertex_t>(multicommodityEdge->getSourceVertex());
        MultiCommodityMinCostFlow::vertex_t::Ptr target = dynamic_pointer_cast<MultiCommodityMinCostFlow::vertex_t>(multicommodityEdge->getTargetVertex());

        for(size_t i = 0; i < commodities; ++i)
        {
            uint32_t initialSetupDemand = target->getCommodityMinTransFlow(i);
            multicommodityEdge->setCommodityCapacityUpperBound(i, initialSetupDemand);
            multicommodityEdge->setCommodityCost(i, getInitialSetupCost(i));
        }
    }
}

graph_analysis::algorithms::MultiCommodityMinCostFlow::edge_t::Ptr MinCostFlow::getPartner(const graph_analysis::BaseGraph::Ptr& flowGraph,
        const Edge::Ptr& edge)
{
    Vertex::Ptr source = mBipartiteGraph.getUniquePartner(edge->getSourceVertex());
    Vertex::Ptr target = mBipartiteGraph.getUniquePartner(edge->getTargetVertex());

    Edge::PtrList edges = flowGraph->getEdges(source, target);
    if(edges.empty())
    {
        throw std::invalid_argument("templ::solvers::transshipment::MinCostFlow::getPartner: could not identify partner edge for '"
                + edge->toString());
    }

    return dynamic_pointer_cast<MultiCommodityMinCostFlow::edge_t>(edges.front());
}

std::vector<Flaw> MinCostFlow::run(bool doThrow)
{
    using namespace graph_analysis;
    using namespace graph_analysis::algorithms;

    uint32_t numberOfCommodities = mCommoditiesRoles.size();
    BaseGraph::Ptr flowGraph = createFlowGraph(numberOfCommodities);
    setCommoditySupplyAndDemand();
    setDepotRestrictions(flowGraph, numberOfCommodities);

    MultiCommodityMinCostFlow minCostFlow(flowGraph, numberOfCommodities, mSolverType);
    // LOGGING
    {
        std::string filename  = mpLogger->filename("multicommodity-min-cost-flow-init.gexf");
        graph_analysis::io::GraphIO::write(filename, flowGraph);
    }

    std::string prefixPath = mpLogger->filename("multicommodity-min-cost-flow");
    algorithms::LPSolver::Status status = minCostFlow.solve(prefixPath);
    switch(status)
    {
        case algorithms::LPSolver::SOLUTION_FOUND:
        case algorithms::LPSolver::STATUS_OPTIMAL:
            break;
        case algorithms::LPSolver::NO_SOLUTION_FOUND:
        case algorithms::LPSolver::STATUS_INFEASIBLE:
        case algorithms::LPSolver::STATUS_UNBOUNDED:
        case algorithms::LPSolver::STATUS_UNKNOWN:
        case algorithms::LPSolver::INVALID_PROBLEM_DEFINITION:
        default:
            if(doThrow)
            {
                throw std::runtime_error("templ::solvers::transshipment::MinCostFlow: no solution found");
            }
    }

    // LOGGING
    {
        std::string filename  = mpLogger->filename("multicommodity-min-cost-flow-final-flow.gexf");
        graph_analysis::io::GraphIO::write(filename, flowGraph);

        filename  = mpLogger->filename("multicommodity-min-cost-flow.gexf");
        minCostFlow.save(filename);
    }

    // Update roles in the space time network using the information of the
    // flow graph
    updateRoles(flowGraph);

    return computeFlaws(minCostFlow);
}

std::vector<Flaw> MinCostFlow::computeFlaws(const MultiCommodityMinCostFlow& minCostFlow) const
{
    std::vector<Flaw> flaws;

    // Check on violations of the current network
    std::vector<ConstraintViolation> violations = minCostFlow.validateInflow();
    LOG_INFO_S << "Violations found: " << violations.size() << " violations in the commodity flow";
    std::vector<ConstraintViolation>::const_iterator vit = violations.begin();
    for(; vit != violations.end(); ++vit)
    {
        const ConstraintViolation& violation = *vit;
        if(! (violation.getType() == graph_analysis::algorithms::ConstraintViolation::TotalTransFlow
                    || violation.getType() == graph_analysis::algorithms::ConstraintViolation::TotalMinFlow
                    || violation.getType() ==
                    graph_analysis::algorithms::ConstraintViolation::FlowBalance) )
        {
            continue;
        }

        // Map violation from multicommodity vertex back to SpaceTime::Network tuple
        Vertex::Ptr spaceTimePartnerVertex = mBipartiteGraph.getUniquePartner(violation.getVertex());
        SpaceTime::Network::tuple_t::Ptr tuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(spaceTimePartnerVertex);

        SpaceTime::Network::value_t location = dynamic_pointer_cast<SpaceTime::Network::value_t::element_type>(tuple->first());

        LOG_INFO_S << "Commodity flow violation: " << violation.toString();

        // Role that is involved into this violation
        // (for TotalTransFlow and TotalMinFlow) this is only
        std::set<uint32_t> commodities = violation.getCommodities();
        Role::List affectedRoles;
        for(uint32_t commodity : commodities)
        {
            const Role& affectedRole = mCommoditiesRoles[commodity];
            affectedRoles.push_back(affectedRole);
        }
        LOG_INFO_S << "Violation for " << Role::toString(affectedRoles) << " -- at: "
                << spaceTimePartnerVertex->toString();

        Flaw flaw(violation, affectedRoles, tuple->getPair());
        flaws.push_back(flaw);
    }

    transshipment::Flaw::List transitionFlaws = mFlowNetwork.getInvalidTransitions(mFeasibilityTimeoutInMs);
    flaws.insert(flaws.begin(), transitionFlaws.begin(), transitionFlaws.end());

    return flaws;
}


void MinCostFlow::updateRoles(const BaseGraph::Ptr& flowGraph)
{
    LOG_DEBUG_S << "Update after flow optimization" << std::endl;
    // TODO: check if this should not be updated at a later state, e.g. after
    // violation processing
    // Update commodities after flow optimization
    EdgeIterator::Ptr edgeIt = flowGraph->getEdgeIterator();
    while(edgeIt->next())
    {
        MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge =
            dynamic_pointer_cast<MultiCommodityMinCostFlow::edge_t>(edgeIt->current());


        for(size_t i = 0; i < mCommoditiesRoles.size(); ++i)
        {
            const Role& role = mCommoditiesRoles[i];
            uint32_t flow = multicommodityEdge->getCommodityFlow(i);
            LOG_DEBUG_S << "Role: " << role.toString() << " flow: " << flow;
            if(flow > 0)
            {
                // Update vertices of mSpaceTimeNetwork
                Vertex::Ptr sourceLocation = mBipartiteGraph.getUniquePartner(multicommodityEdge->getSourceVertex());
                Vertex::Ptr targetLocation = mBipartiteGraph.getUniquePartner(multicommodityEdge->getTargetVertex());
                assert(sourceLocation && targetLocation);

                LOG_DEBUG_S << "Add role " << role.toString() << std::endl
                    << "    source: " << sourceLocation->toString() << multicommodityEdge->getSourceVertex()->toString() << std::endl
                    << "    target: " << targetLocation->toString() << multicommodityEdge->getTargetVertex()->toString() << std::endl
                ;

                dynamic_pointer_cast<SpaceTime::Network::tuple_t>(sourceLocation)->addRole(role, RoleInfo::ASSIGNED);
                dynamic_pointer_cast<SpaceTime::Network::tuple_t>(targetLocation)->addRole(role, RoleInfo::ASSIGNED);

                Edge::PtrList edges = mSpaceTimeNetwork.getGraph()->getEdges(sourceLocation, targetLocation);
                if(edges.empty())
                {
                    throw std::runtime_error("templ::solvers::transshipment::MinCostFlow::updateRoles: "
                            " no edge in corresponding SpaceTime Network");

                } else if(edges.size() > 1)
                {
                    throw std::runtime_error("templ::solvers::transshipment::MinCostFlow::updateRoles: "
                            " multi-edge in corresponding SpaceTime Network");
                } else {
                    RoleInfo::Ptr roleInfo = dynamic_pointer_cast<RoleInfo>(edges.back());
                    roleInfo->addRole(role, RoleInfo::ASSIGNED);
                }
            }
        }
    }
}

SpaceTime::Network::tuple_t::Ptr MinCostFlow::getFromTimeTuple(const FluentTimeResource& ftr)
{
    // Map back space and time to human-readable information
    const symbols::constants::Location::Ptr& location = ftr.getLocation();
    const solvers::temporal::Interval& interval = ftr.getInterval();

    // Get the tuple in the graph and augment with role
    // information
    return mSpaceTimeNetwork.tupleByKeys(location, interval.getFrom());
}

SpaceTime::Network::tuple_t::Ptr MinCostFlow::getToTimeTuple(const FluentTimeResource& ftr)
{
    // Map back space and time to human-readable information
    const symbols::constants::Location::Ptr& location = ftr.getLocation();
    const solvers::temporal::Interval& interval = ftr.getInterval();

    // Get the tuple in the graph and augment with role
    // information
    return mSpaceTimeNetwork.tupleByKeys(location, interval.getTo());
}

MultiCommodityMinCostFlow::vertex_t::Ptr MinCostFlow::getPartner(const SpaceTime::Network::tuple_t::Ptr& tuple)
{
    Vertex::Ptr vertex = mBipartiteGraph.getUniquePartner(tuple);

    if(!vertex)
    {
        throw std::invalid_argument("templ::solvers::transshipment::MinCostFlow::getToTimePartner: "
                "failed to get corresponding vertex for resource: " + tuple->toString());
    }
    return dynamic_pointer_cast<MultiCommodityMinCostFlow::vertex_t>(vertex);
}

void MinCostFlow::setMinTransFlow(const SpaceTime::Network::tuple_t::Ptr& tuple,
        uint32_t commodityId,
        const Role& role,
        uint32_t value
        )
{
    MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex = getPartner(tuple);
    // intermediate ones
    // set minimum flow that needs to go through this node
    // for this commodity (i.e. can be either 0 or 1)
    multicommodityVertex->setCommodityMinTransFlow(commodityId, value);
    tuple->addRole(role, RoleInfo::REQUIRED);
}

void MinCostFlow::setSupplyDemand(const SpaceTime::Network::tuple_t::Ptr& tuple,
        uint32_t commodityId,
        const Role& role,
        int32_t value)
{
    MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex = getPartner(tuple);
    // intermediate ones
    // set minimum flow that needs to go through this node
    // for this commodity (i.e. can be either 0 or 1)
    multicommodityVertex->setCommoditySupply(commodityId, value);
    if(value >= 0)
    {
        tuple->addRole(role, RoleInfo::AVAILABLE);
    } else {
        tuple->addRole(role, RoleInfo::REQUIRED);
    }
}

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
