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
    MinCostFlow(
            const std::map<Role, csp::RoleTimeline>& expandedTimelines,
            const std::map<Role, csp::RoleTimeline>& minimalTimelines,
            const symbols::constants::Location::PtrList& locations,
            const temporal::point_algebra::TimePoint::PtrList& sortedTimepoints,
            const organization_model::OrganizationModelAsk& ask,
            const utils::Logger::Ptr& logger,
            graph_analysis::algorithms::LPSolver::Type solverType = graph_analysis::algorithms::LPSolver::GLPK_SOLVER,
            double feasibilityTimeoutInMs = 1000
            );

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
     * Set the restrictions on the depot outgoing arcs, in order direct flow
     * to initially demanded
     */
    void setDepotRestrictions(const graph_analysis::BaseGraph::Ptr& flowGraph,
            uint32_t commodities);

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

    SpaceTime::Network::tuple_t::Ptr getFromTimeTuple(const FluentTimeResource& ftr);
    SpaceTime::Network::tuple_t::Ptr getToTimeTuple(const FluentTimeResource& ftr);

    graph_analysis::algorithms::MultiCommodityMinCostFlow::vertex_t::Ptr getPartner(const SpaceTime::Network::tuple_t::Ptr& tuple);
    graph_analysis::algorithms::MultiCommodityMinCostFlow::edge_t::Ptr getPartner(const graph_analysis::BaseGraph::Ptr& flowGraph, const graph_analysis::Edge::Ptr& edge);

    /**
     * Set the commodity supply demand for a tuple, based on the TimepointType
     */
    void setMinTransFlow(const SpaceTime::Network::tuple_t::Ptr& tuple,
            uint32_t commodityId,
            const Role& role,
            uint32_t value
            );

    void setSupplyDemand(const SpaceTime::Network::tuple_t::Ptr& tuple,
            uint32_t commodityId,
            const Role& role,
            int32_t value
            );

    /**
     * Allow to define the initial setup cost for a particular commodity
     */
    void setInitialSetupCost(uint32_t commodity, double cost) { mInitialSetupCost[commodity] = cost; }

    /**
     * Allow to retrieve the initial setup cost to be set for the optimization
     * problem
     * Initial setup cost should not be zero
     * \return intial setup cost
     */
    uint32_t getInitialSetupCost(uint32_t commodity) const { return mInitialSetupCost[commodity]; }

private:
    std::map<Role, csp::RoleTimeline> mExpandedTimelines;
    std::map<Role, csp::RoleTimeline> mMinRequiredTimelines;

    temporal::point_algebra::TimePoint::PtrList mSortedTimepoints;

    organization_model::OrganizationModelAsk mAsk;
    utils::Logger::Ptr mpLogger;
    FlowNetwork mFlowNetwork;

    std::vector<Role> mCommoditiesRoles;
    std::vector<double> mInitialSetupCost;

    SpaceTime::Network mSpaceTimeNetwork;
    // Store the mapping between flow graph and space time network
    graph_analysis::BipartiteGraph mBipartiteGraph;
    graph_analysis::algorithms::LPSolver::Type mSolverType;

    double mFeasibilityTimeoutInMs;
};

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVER_
