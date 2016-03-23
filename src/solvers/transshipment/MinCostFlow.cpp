#include "MinCostFlow.hpp"
#include <base/Logging.hpp>
#include <graph_analysis/BipartiteGraph.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <organization_model/facets/Robot.hpp>
#include <templ/solvers/csp/FluentTimeResource.hpp>
#include <templ/SpaceTimeNetwork.hpp>
#include <templ/Logger.hpp>

using namespace graph_analysis;
using namespace graph_analysis::algorithms;

namespace templ {
namespace solvers {
namespace transshipment {

MinCostFlow::MinCostFlow(const Mission& mission,
        const std::map<Role, csp::RoleTimeline>& timelines,
        SpaceTimeNetwork* spaceTimeNetwork)
    : mMission(mission)
    , mTimelines(timelines)
    , mSpaceTimeNetwork(mission.getLocations(), mission.getTimepoints())
{

    std::map<Role, csp::RoleTimeline>::const_iterator rit = mTimelines.begin();
    for(; rit != mTimelines.end(); ++rit)
    {
        const Role& role = rit->first;
        organization_model::facets::Robot robot(role.getModel(), mMission.getOrganizationModelAsk());
        if(!robot.isMobile()) // only immobile systems are relevant
        {
            // Allow to later map roles back from index
            mCommoditiesRoles.push_back(role);
        }
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
            // Retrieve the sorted list of FluentTimeResources
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
                SpaceTimeNetwork::tuple_t::Ptr currentTuple = mSpaceTimeNetwork.tupleByKeys(location, interval.getFrom());
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

std::vector<Flaw> MinCostFlow::compute()
{
    using namespace graph_analysis;
    using namespace graph_analysis::algorithms;

    uint32_t numberOfCommodities = mCommoditiesRoles.size();
    BaseGraph::Ptr flowGraph = createFlowGraph(numberOfCommodities);
    setCommoditySupplyAndDemand();

    MultiCommodityMinCostFlow minCostFlow(flowGraph, numberOfCommodities);
    // LOGGING
    {
        std::string filename  = mMission.getLogger()->filename("min-cost-flow-init.dot");
        graph_analysis::io::GraphIO::write(filename, flowGraph);
    }

    uint32_t cost = minCostFlow.run();
    LOG_DEBUG_S << "Ran flow optimization: min cost: " << cost << std::endl;
    minCostFlow.storeResult();

    // LOGGING
    {
        std::string filename  = mMission.getLogger()->filename("min-cost-flow-result.dot");
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
        // Map violation from multicommodity vertex back to SpaceTimeNetwork tuple
        Vertex::Ptr spaceTimePartnerVertex = mBipartiteGraph.getUniquePartner(violation.getVertex());
        SpaceTimeNetwork::tuple_t::Ptr tuple = dynamic_pointer_cast<SpaceTimeNetwork::tuple_t>(spaceTimePartnerVertex);

        SpaceTimeNetwork::value_t::Ptr location = dynamic_pointer_cast<SpaceTimeNetwork::value_t>(tuple->first());

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

        Flaw flaw(violation, affectedRole);

        std::vector<csp::FluentTimeResource>::const_iterator fit = getFluent(roleTimeline, tuple);
        switch(violation.getType())
        {
            // In the case of transflow we add a triebreaker between the current
            // and the subsequent requirement
            case ConstraintViolation::TransFlow:
            {
                csp::FluentTimeResource ftr = *fit;
                csp::FluentTimeResource ftr_subsequent = *(fit+1);

                flaw.ftr = *fit;
                flaw.subsequentFtr = *(fit+1);

/*
                LOG_INFO_S << "Transflow violation in timeline: enforce distiction on timeline for: " << affectedRole.toString()
                    << " between " << std::endl
                    << ftr.toString() << " and " << ftr_subsequent.toString() << std::endl
                    << ftr.getInterval(mission).toString() << std::endl
                    << ftr_subsequent.getInterval(mission).toString() << std::endl
                    << " timeline: " << std::endl
                    << roleTimeline.toString()
                    << std::endl;

                organization_model::facets::Robot robot(affectedRole.getModel(), mMission.getOrganizationModelAsk());
                if(!robot.isMobile())
                {
                    // More transport availability needed
                    // TODO: quantify request, e.g. for 1 more Payload
                    Resolver::Ptr functionalityRequest(new FunctionalityRequest(location, ftr.getInterval(mission), vocabulary::OM::resolve("TransportService")));
                    state->addResolver(functionalityRequest);
                }

                 // delta needs to be: missing count of resource type
                 assert(-violation.getDelta() > 0);
                 Resolver::Ptr roleAddDistinction(new RoleAddDistinction(ftr, ftr_subsequent, affectedRole.getModel(), abs( violation.getDelta() ), state->getRoleDistributionSolution()));
                 state->addResolver(roleAddDistinction);
*/
                break;
            }
            // In the case of minflow we add a triebreaker between the current
            // and the previous requirement
            case ConstraintViolation::MinFlow:
            {
                flaw.ftr = *fit;
                flaw.previousFtr = *(fit-1);
/*
                FluentTimeResource ftr = *fit;
                FluentTimeResource ftr_previous = *(fit-1);
                LOG_INFO_S << "Minflow violation in timeline: enforce distiction on timeline for: " << affectedRole.toString()
                    << " between " << std::endl
                    << ftr_previous.toString() << " and " << ftr.toString() << std::endl
                    << ftr_previous.getInterval(mission).toString() << std::endl
                    << ftr.getInterval(mission).toString() << std::endl
                    << " timeline: " << std::endl
                    << roleTimeline.toString()
                    << std::endl;

                organization_model::facets::Robot robot(affectedRole.getModel(), mOrganizationModelAsk);
                if(!robot.isMobile())
                {
                    std::cout << "Robot is not mobile thus requesting a TransportService" << std::endl;
                   // More transport availability needed
                   // TODO: quantify request, e.g. for 1 more Payload
                   Resolver::Ptr functionalityRequest(new FunctionalityRequest(location, ftr.getInterval(mission), vocabulary::OM::resolve("TransportService")));
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

        for(size_t i = 0; i < mCommoditiesRoles.size(); ++i)
        {
            uint32_t flow = multicommodityEdge->getCommodityFlow(i);
            if(flow > 0)
            {
                const Role& role = mCommoditiesRoles[i];
                // Update vertices of mSpaceTimeNetwork
                Vertex::Ptr sourceLocation = mBipartiteGraph.getUniquePartner(multicommodityEdge->getSourceVertex());
                Vertex::Ptr targetLocation = mBipartiteGraph.getUniquePartner(multicommodityEdge->getTargetVertex());
                assert(sourceLocation && targetLocation);

                dynamic_pointer_cast<SpaceTimeNetwork::tuple_t>(sourceLocation)->addRole(role);
                dynamic_pointer_cast<SpaceTimeNetwork::tuple_t>(targetLocation)->addRole(role);
            }
        }
    }
}

std::vector<csp::FluentTimeResource>::const_iterator MinCostFlow::getFluent(const csp::RoleTimeline& roleTimeline, const SpaceTimeNetwork::tuple_t::Ptr& tuple) const
{
    LOG_WARN_S << "Find tuple: " << tuple->toString() << " in timeline " << roleTimeline.toString();

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