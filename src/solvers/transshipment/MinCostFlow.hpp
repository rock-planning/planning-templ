#ifndef TEMPL_SOLVER_TRANSSHIPMENT_MINCOSTFLOW_HPP
#define TEMPL_SOLVER_TRANSSHIPMENT_MINCOSTFLOW_HPP

#include <vector>
#include <graph_analysis/BipartiteGraph.hpp>
#include <graph_analysis/algorithms/MultiCommodityMinCostFlow.hpp>
#include <templ/Mission.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>
#include <templ/solvers/transshipment/FlowNetwork.hpp>
#include "Flaw.hpp"

namespace templ {
namespace solvers {
namespace transshipment {

struct MinCostFlowStatus
{
    graph_analysis::BaseGraph::Ptr flowGraph;
    std::vector<graph_analysis::algorithms::ConstraintViolation> violations;
    uint32_t commodities;
};

class MinCostFlow
{
public:
    /**
     * Initialize the basic min cost flow problem using an existing mission
     * \param mission basic mission we try to solve
     * \param minimalTimelines All role based timelines that fulfill the minimum
     * requirements
     * \param expandedTimeline optional timelines which serve as a guide for mobile system flow
     * result of the multi-commodity min-cost flow optimization
     */
    MinCostFlow(const Mission::Ptr& mission,
            const std::map<Role, csp::RoleTimeline>& minimalTimelines,
            const SpaceTime::Timelines& expandedTimelines = SpaceTime::Timelines());


    /**
     * Run the min cost flow optimization and return the list of flaws found in
     * this solution
     * \param doThrow throw when the underlying simplex does not a return a
     * proper solution
     * \return flaws found
     */
    std::vector<Flaw> run(bool doThrow = false);

    FlowNetwork& getFlowNetwork() { return mFlowNetwork; }
protected:
    /**
     *  Translating the space time network into the mincommodity representation,
     *  i.e. the flow graph
     *  This will fill the BipartiteGraph to allow the mapping of the space-time graph
     *  to the flow graph
     *
     *  The flow graph will contain vertices of the type
     *  graph_analysis::algorithms::MultiCommodityMinCostFlow::edge_t::Ptr
     *  and
     *  graph_analysis::algorithms::MultCommodityMinCostFlow::vertex_t::Ptr
     *
     * \param commodities Number of commodities that should be taken into
     * consideration for the underlying MultiCommodityMinCostFlow problem
     */
    graph_analysis::BaseGraph::Ptr createFlowGraph(uint32_t commodities);

    /**
     *  Set the commodity supply and demand
     *  Since the general transport network is constructed from mobile systems,
     *  that means supply and demand comes from the requirements of immobile systems.
     *  This function thus sets the  'start','end' and '(transition) waypoints' for all
     *  immobile systems
     *
     *  Operates on the flow graph through the mapping of the bipartite graph
     */
    void setCommoditySupplyAndDemand();

    /**
     * After the flow optimization has taken place -- update the space time
     * network, i.e. the locations, with information about the roles
     * Update the spaceTimeNetwork from the data of the flowGraph using the reverse mapping and adding
     * the corresponding (and new) roles
     */
    void updateRoles(const graph_analysis::BaseGraph::Ptr& flowGraph);

    /**
     * Analyse the result of the optimization and identify the current
     * (partial) solution upon flaws
     * \return List of existing flaws in the solution
     */
    std::vector<Flaw> computeFlaws(const graph_analysis::algorithms::MultiCommodityMinCostFlow&) const;

    /**
     * Find the Fluent which corresponds to the given network tuple to allow
     * a reverse mapping between gaph based representation and fluents
     */
    std::vector<templ::solvers::FluentTimeResource>::const_iterator
        getFluent(const templ::solvers::csp::RoleTimeline& roleTimeline,
            const SpaceTime::Network::tuple_t::Ptr& tuple) const;

    SpaceTime::Network::tuple_t::Ptr getFromTimeTuple(const FluentTimeResource& ftr);
    SpaceTime::Network::tuple_t::Ptr getToTimeTuple(const FluentTimeResource& ftr);

    graph_analysis::algorithms::MultiCommodityMinCostFlow::vertex_t::Ptr getPartner(const SpaceTime::Network::tuple_t::Ptr& tuple);

private:
    Mission::Ptr mpMission;
    std::map<Role, csp::RoleTimeline> mTimelines;
    SpaceTime::Timelines mExpandedTimelines;
    std::vector<Role> mCommoditiesRoles;

    FlowNetwork mFlowNetwork;
    SpaceTime::Network mSpaceTimeNetwork;
    // Store the mapping between flow graph and space time network
    graph_analysis::BipartiteGraph mBipartiteGraph;
};

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVER_
