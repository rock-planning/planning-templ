#include "MinCostFlow.hpp"
#include <base-logging/Logging.hpp>
#include <graph_analysis/BipartiteGraph.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <organization_model/facets/Robot.hpp>
#include <templ/solvers/csp/FluentTimeResource.hpp>
#include <templ/utils/Logger.hpp>

using namespace graph_analysis;
using namespace graph_analysis::algorithms;

namespace templ {
namespace solvers {
namespace transshipment {

MinCostFlow::MinCostFlow(const Mission::Ptr& mission,
        const std::map<Role, csp::RoleTimeline>& timelines,
        const SpaceTime::Timelines& expandedTimelines)
    : mpMission(mission)
    , mTimelines(timelines)
    , mExpandedTimelines(expandedTimelines)
    , mFlowNetwork(mission, timelines, expandedTimelines)
    , mSpaceTimeNetwork(mFlowNetwork.getSpaceTimeNetwork())
{

    std::map<Role, csp::RoleTimeline>::const_iterator rit = mTimelines.begin();
    for(; rit != mTimelines.end(); ++rit)
    {
        const Role& role = rit->first;
        organization_model::facets::Robot robot(role.getModel(), mpMission->getOrganizationModelAsk());
        if(!robot.isMobile()) // only immobile systems are relevant
        {
            // Allow to later map roles back from index
            mCommoditiesRoles.push_back(role);
        }
    }
    if(mCommoditiesRoles.empty())
    {
        LOG_WARN_S << "No immobile systems, thus no commodities to be routed";
    }
}

BaseGraph::Ptr MinCostFlow::createFlowGraph(uint32_t commodities)
{
   BaseGraph::Ptr flowGraph = BaseGraph::getInstance();

   // Create the vertices of the flow, that can be mapped back to the space time
   // network using the bipartiteGraph structure
   VertexIterator::Ptr vertexIt = mSpaceTimeNetwork.getGraph()->getVertexIterator();
   while(vertexIt->next())
   {
       MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex(new MultiCommodityMinCostFlow::vertex_t(commodities));
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

       // Create an edge in the multicommodity representation that correspond to the space time graph
       MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge(new MultiCommodityMinCostFlow::edge_t(commodities));
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
           multicommodityEdge->setCommodityCapacityUpperBound(i, bound);
       }
       flowGraph->addEdge(multicommodityEdge);
   }

   return flowGraph;
}

void MinCostFlow::setCommoditySupplyAndDemand()
{
    std::map<Role, csp::RoleTimeline>::const_iterator rit = mTimelines.begin();
    for(; rit != mTimelines.end(); ++rit)
    {
        const Role& role = rit->first;
        const csp::RoleTimeline& roleTimeline = rit->second;

        // The list of commodities roles is initialized in the constructor and
        // contains the list of immobile units
        std::vector<Role>::const_iterator cit = std::find(mCommoditiesRoles.begin(), mCommoditiesRoles.end(), role);
        if(cit != mCommoditiesRoles.end())
        {
            size_t commodityId = cit - mCommoditiesRoles.begin();
            // Retrieve the (time-based) sorted list of FluentTimeResources
            const std::vector<csp::FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
            std::vector<csp::FluentTimeResource>::const_iterator fit = ftrs.begin();
            Vertex::Ptr previous;
            for(; fit != ftrs.end(); ++fit)
            {
                // Map back space and time to human-readable information
                symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
                solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

                // Get the tuple in the graph and augment with role
                // information
                SpaceTime::Network::tuple_t::Ptr currentTuple = mSpaceTimeNetwork.tupleByKeys(location, interval.getFrom());
                currentTuple->addRole(role);

                Vertex::Ptr vertex = mBipartiteGraph.getUniquePartner(currentTuple);
                assert(vertex);
                MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex =
                    dynamic_pointer_cast<MultiCommodityMinCostFlow::vertex_t>(vertex);

                // - first entry (start point): can be interpreted as source
                // - intermediate entries (transflow) as lower flow bound
                // - final entry as demand
                if(fit == ftrs.begin())
                {
                    multicommodityVertex->setCommoditySupply(commodityId, 1);
                } else if(fit+1 == ftrs.end())
                {
                    multicommodityVertex->setCommoditySupply(commodityId, -1);
                } else { // intermediate ones
                    // set minimum flow that needs to go through this node
                    // for this commodity (i.e. can be either 0 or 1)
                    multicommodityVertex->setCommodityMinTransFlow(commodityId, 1);
                }
                previous = vertex;
            }
        }
    } // end for role timelines
}

std::vector<Flaw> MinCostFlow::run()
{
    using namespace graph_analysis;
    using namespace graph_analysis::algorithms;

    uint32_t numberOfCommodities = mCommoditiesRoles.size();
    BaseGraph::Ptr flowGraph = createFlowGraph(numberOfCommodities);
    setCommoditySupplyAndDemand();

    MultiCommodityMinCostFlow minCostFlow(flowGraph, numberOfCommodities);
    // LOGGING
    {
        std::string filename  = mpMission->getLogger()->filename("min-cost-flow-init.dot");
        graph_analysis::io::GraphIO::write(filename, flowGraph);
    }

    uint32_t cost = minCostFlow.run();
    LOG_DEBUG_S << "Ran flow optimization: min cost: " << cost << std::endl;
    minCostFlow.storeResult();

    // LOGGING
    {
        std::string filename  = mpMission->getLogger()->filename("min-cost-flow-result.dot");
        graph_analysis::io::GraphIO::write(filename, flowGraph);
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
    LOG_INFO_S << "Violoations found: " << violations.size() << " violations in the commodity flow";
    std::vector<ConstraintViolation>::const_iterator vit = violations.begin();
    for(; vit != violations.end(); ++vit)
    {
        const ConstraintViolation& violation = *vit;
        // Role that is involved into this violation
        const Role& affectedRole = mCommoditiesRoles[violation.getCommodity()];
        // Map violation from multicommodity vertex back to SpaceTime::Network tuple
        Vertex::Ptr spaceTimePartnerVertex = mBipartiteGraph.getUniquePartner(violation.getVertex());
        SpaceTime::Network::tuple_t::Ptr tuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(spaceTimePartnerVertex);

        SpaceTime::Network::value_t location = dynamic_pointer_cast<SpaceTime::Network::value_t::element_type>(tuple->first());

        LOG_INFO_S << "Commodity flow violation: " << violation.toString();
        LOG_INFO_S << "Violation for " << affectedRole.toString() << " -- at: "
            << spaceTimePartnerVertex->toString();

        std::map<Role, csp::RoleTimeline>::const_iterator rit = mTimelines.find(affectedRole);
        if(rit == mTimelines.end())
        {
            throw std::runtime_error("templ::solvers::transshipment::MinCostFlow::computeFlaws: "
                    " failed to find role timeline for affectedRole: " + affectedRole.toString());
        }
        const csp::RoleTimeline& roleTimeline = rit->second;


        std::vector<csp::FluentTimeResource>::const_iterator fit = getFluent(roleTimeline, tuple);

        Flaw flaw(violation, affectedRole);
        flaw.ftr = *fit;

        switch(violation.getType())
        {
            // In the case of transflow we add a triebreaker between the current
            // and the subsequent requirement
            case ConstraintViolation::TransFlow:
            {
                flaw.subsequentFtr = *(fit+1);

                ////organization_model::facets::Robot robot(affectedRole.getModel(), mpMission->getOrganizationModelAsk());
                ////if(!robot.isMobile())
                ////{
                ////    // More transport availability needed
                ////    // TODO: quantify request, e.g. for 1 more Payload
                ////    Resolver::Ptr functionalityRequest(new FunctionalityRequest(location, ftr.getInterval(mission), vocabulary::OM::resolve("FlowProvider")));
                ////    state->addResolver(functionalityRequest);
                ////}

                //// // delta needs to be: missing count of resource type
                //// assert(-violation.getDelta() > 0);
                //// Resolver::Ptr roleAddDistinction(new RoleAddDistinction(ftr, ftr_subsequent, affectedRole.getModel(), abs( violation.getDelta() ), state->getRoleDistributionSolution()));
                //// state->addResolver(roleAddDistinction);
                break;
            }
            // In the case of minflow we add a triebreaker between the current
            // and the previous requirement
            case ConstraintViolation::MinFlow:
            {
                flaw.previousFtr = *(fit-1);
/*
                organization_model::facets::Robot robot(affectedRole.getModel(), mOrganizationModelAsk);
                if(!robot.isMobile())
                {
                    std::cout << "Robot is not mobile thus requesting a FlowProvider" << std::endl;
                   // More transport availability needed
                   // TODO: quantify request, e.g. for 1 more Payload
                   Resolver::Ptr functionalityRequest(new FunctionalityRequest(location, ftr.getInterval(mission), vocabulary::OM::resolve("FlowProvider")));
                   state->addResolver(functionalityRequest);
                }

                // delta needs to be: missing count of resource type
                assert(-violation.getDelta() > 0);
                Resolver::Ptr roleAddDistinction(new RoleAddDistinction(ftr_previous, ftr, affectedRole.getModel(), abs( violation.getDelta() ), state->getRoleDistributionSolution()));
                state->addResolver(roleAddDistinction);
    */
            }
            break;
        }
        flaws.push_back(flaw);
    }

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

        LOG_DEBUG_S << "Edge: " << multicommodityEdge->toString();

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
            }
        }
    }
}

std::vector<csp::FluentTimeResource>::const_iterator MinCostFlow::getFluent(const csp::RoleTimeline& roleTimeline, const SpaceTime::Network::tuple_t::Ptr& tuple) const
{
    LOG_DEBUG_S << "Find tuple: " << tuple->toString() << " in timeline " << roleTimeline.toString();

    const std::vector<csp::FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
    std::vector<csp::FluentTimeResource>::const_iterator fit = ftrs.begin();
    for(; fit != ftrs.end(); ++fit)
    {
        symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
        solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

        if(location == tuple->first() && interval.getFrom() == tuple->second())
        {
            return fit;
        }
    }
    throw std::invalid_argument("MissionPlanner::getFluent: could not retrieve corresponding fluent in timeline: '"
            + roleTimeline.toString() + "' from tuple '" + tuple->toString());
}

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
