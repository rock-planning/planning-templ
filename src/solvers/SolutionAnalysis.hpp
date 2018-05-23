#ifndef TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP
#define TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP

#include <qxcfg/Configuration.hpp>
#include "../Mission.hpp"
#include "../SpaceTime.hpp"
#include "Solution.hpp"
#include "FluentTimeResource.hpp"
#include "../Plan.hpp"
#include "temporal/TemporalConstraintNetwork.hpp"
#include <organization_model/Analyser.hpp>

namespace templ {
namespace solvers {

/**
 * Allow to analyse an existing solution upon quality, cost, and
 * redundancy
 */
class SolutionAnalysis
{
public:
    typedef std::pair< organization_model::ModelPool::List, organization_model::ModelPool::List >
        MinMaxModelPools;

    /**
     * An existing solution will provide contain RoleInfoTuple as vertices
     * and An existing solution will provide contain RoleInfoTuple as vertices
     * and RoleInfoWeightedEdge as edges
     *
     * The successful assignment of roles to a vertex will result in a entry of
     * the RoleInfo::ASSIGNED tagged set
     * Meanwhile the default set, contains all the required roles
     * Note, that for the first timepoint the starting requirement vs. assiged have to be
     * interpreted differently, i.e. the requirements are merley defining what
     * is available at this 'source' hub
     *
     * \see SpaceTime::Network
     */
    SolutionAnalysis(const Mission::Ptr& mission, const SpaceTime::Network& solution,
            qxcfg::Configuration configuration = qxcfg::Configuration());

    SolutionAnalysis(const Mission::Ptr& mission,
            const graph_analysis::BaseGraph::Ptr& graph,
            qxcfg::Configuration configuration = qxcfg::Configuration());

    void updateAnalyser();

    void analyse();

    void save(const std::string& filename = "") const;

    double getQuality() const { return mQuality; }
    double getCost() const { return mCost; }
    double getSafety() const { return mSafety; }
    double getEfficacy() const { return getCost(); }
    double getReconfigurationCost() const { return mReconfigurationCost; }
    double getTravelledDistance() const { return mTraveledDistance; }

    /**
     * Get the metrics, e.g., redundancy of the fluent time resource
     */
    double getSafety(const FluentTimeResource& ftr) const;

    double getSafety(const FluentTimeResource& ftr,
            const SpaceTime::Network::tuple_t::Ptr& tuple) const;

    /**
     * Get the metric value for minimum requirement and minimum available
     * resources
     */
    double getSafety(const organization_model::ModelPool& minRequired,
            const organization_model::ModelPool& minAvailable) const;

    /**
     * Retrieve the list of required roles / all roles that are involved in this
     * problem
     *
     * A required role is given, it its usage exceeds 1 (since the initial usage
     * is given by its availability at the initial position)
     */
    std::set<Role> getRequiredRoles(size_t minRequirement = 1) const;

    /**
     * Get the minimum required resource for the given fluent time resource
     * \return model pool of the minimum required resources
     */
    organization_model::ModelPool getMinResourceRequirements(const FluentTimeResource& ftr) const;

    /**
     * Get the maximum required resource for the given fluent time resource
     * \return model pool of the maximum required resources
     */
    organization_model::ModelPool getMaxResourceRequirements(const FluentTimeResource& ftr) const;

    /**
     * Get the corresponding space time tuple for source time of a fluent time resource
     * definition
     */
    SpaceTime::Network::tuple_t::Ptr getFromTuple(const FluentTimeResource& ftr) const;

    /**
     * Get the corresponding space time tuple for the target time of a fluent time resource
     * definition
     */
    SpaceTime::Network::tuple_t::Ptr getToTuple(const FluentTimeResource& ftr) const;

    /**
     * Get the maximum number of missing resource requirements, i.e.
     * required (by original mission definition) resources - minimum available resources
     *
     * This takes into account resolution of functionality to actual agents to
     * come to a particular solution
     */
    organization_model::ModelPoolDelta getMinMissingResourceRequirements(const FluentTimeResource& ftr) const;

    /**
     * Get the minimum number of missing resource requirements, i.e.
     * required (by original mission definition) resources - maximum available resources
     *
     * This takes into account resolution of functionality to actual agents to
     * come to a particular solution
     */
    organization_model::ModelPoolDelta getMaxMissingResourceRequirements(const FluentTimeResource& ftr) const;

    /**
     * Get the maximum number of missing resources, i.e.
     * required (by transformed mission definition) resources - minimum available resources
     *
     * This takes into account resolution of functionality to actual agents to
     * come to a particular solution
     */
    organization_model::ModelPoolDelta getMaxMissingResources(const FluentTimeResource& ftr) const;

    /**
     * Get the minimum number of missing resources, i.e.
     * required resources - maximum available resources
     *
     * This takes into account resolution of functionality to actual agents to
     * come to a particular solution
     */
    organization_model::ModelPoolDelta getMinMissingResources(const FluentTimeResource& ftr) const;

    /**
     * Get the minimum number of available resources for the given fluent time
     * resource definition
     * \return ModelPool containing the available resources (including inferred
     * functionalities)
     */
    organization_model::ModelPool getMinAvailableResources(const FluentTimeResource& ftr) const;

    organization_model::ModelPool getMinAvailableResources(const SpaceTime::Network::tuple_t::Ptr& tuple) const;

    /**
     * Get the maximum number of available resources for the given fluent time
     * resource definition
     *
     * This function includes all infered functionality
     * \return ModelPool containing the available resources (including inferred
     * functionalities)
     */
    organization_model::ModelPool getMaxAvailableResources(const FluentTimeResource& ftr) const;

    /**
     * Get the required resources as a pair of two model pool list for min
     * cardinalities and max cardinalities
     */
    MinMaxModelPools getRequiredResources(const symbols::constants::Location::Ptr& location, const solvers::temporal::Interval& interval) const;

    /**
     * Get the required resource for an existing fluent time resource from a
     * solution, i.e.
     * find the corresponding match in the mission description
     */
    MinMaxModelPools getRequiredResources(const FluentTimeResource& ftr) const;

    /**
     * Get the availability as list of model pools over the course of one interval
     * This accounts for all included (known) qualitative timepoints
     */
    std::vector<organization_model::ModelPool> getAvailableResources(const symbols::constants::Location::Ptr& location, const solvers::temporal::Interval& interval) const;

    organization_model::ModelPool getAvailableResources(const symbols::constants::Location::Ptr& location, const solvers::temporal::point_algebra::TimePoint::Ptr& timepoint) const;
    /**
     * Compute a hypergraph
     * The hypergaph contains a number of RoleInfoVertex (as HyperEdge)
     * which are linked to by 'requires' Edges from existing edges
     */
    graph_analysis::BaseGraph::Ptr toHyperGraph();

    /**
     * Get the solution network
     */
    const SpaceTime::Network& getSolutionNetwork() const { return mSolutionNetwork; }

    // Annotation functions
    /**
     * Provide a quantification on the transition times for this planner
     * this update the Time Distance Graph
     */
    void quantifyTime();

    // End annotation functions

    /**
     * Get the set of available resources
     */
    //organization_model::ModelPool getAvailableResources(const solvers::FluentTimeResource& e) const;

    std::string toString(size_t indent = 0) const;

    /**
     * Compute a plan for all robot systems
     * The empty role will be used to collect all requirements as
     * openRequirements
     */
    Plan computePlan() const;

    graph_analysis::BaseGraph::Ptr getTimeDistanceGraph() const { return mpTimeDistanceGraph; }

    /**
     * Compute efficacy as function of satisfiability
     \f[
         E = \frac{1}{|STR|} \sum_{s\inSTR} sat(s)
     \f]
     * The resulting value can be retrieve with getEfficiency
     */
    void computeEfficacy();

    /**
     * Compute efficiency as overall energy cost
     */
    void computeEfficiency();

    /**
     * Compute reconfiguration cost for all reconfigurations
     * involved in the process
     */
    void computeReconfigurationCost();

    /**
     * Compute the reconfiguration cost at one vertex
     * (RoleInfoTuple connected with RoleInfoWeightedEdge)
     */
    double computeReconfigurationCost(const graph_analysis::Vertex::Ptr& vertex,
            const graph_analysis::BaseGraph::Ptr& graph);

    void computeSafety();

    std::string getRowDescriptor() const;
    std::string toRow() const;

private:
    double degreeOfFulfillment(const solvers::FluentTimeResource& requirement);

    Mission::Ptr mpMission;
    SpaceTime::Network mSolutionNetwork;
    Plan mPlan;

    std::vector<solvers::FluentTimeResource> mResourceRequirements;
    solvers::temporal::point_algebra::TimePointComparator mTimepointComparator;

    mutable graph_analysis::BaseGraph::Ptr mpTimeDistanceGraph;

    double mAlpha;
    double mBeta;
    double mSigma;

    double mCost;
    solvers::temporal::TemporalConstraintNetwork::Assignment mTimeAssignment;
    double mTimeHorizonInS;

    double mQuality;
    double mSafety;
    double mEfficacy;
    /// The overall cost of the mission
    double mEfficiency;
    double mTraveledDistance;
    double mReconfigurationCost;
    /// The cost per role
    std::map<Role, double> mEfficiencyPerRole;

    organization_model::Analyser mAnalyser;

    qxcfg::Configuration mConfiguration;

};

} // end namespace solvers
} // end namespace templ
#endif // TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP
