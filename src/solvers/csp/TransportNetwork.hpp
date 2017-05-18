#ifndef TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP
#define TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP

#include <string>
#include <map>
#include <vector>
#include <gecode/set.hh>
#include <gecode/search.hh>

#include <organization_model/OrganizationModelAsk.hpp>

#include "../../Mission.hpp"
#include "Types.hpp"
#include "FluentTimeResource.hpp"
#include "utils/FluentTimeIndex.hpp"
#include "../transshipment/MinCostFlow.hpp"

namespace templ {
namespace solvers {
namespace csp {

/**
 * \class TransportNetwork
 * \details TransportNetwork tries to associate actor models
 * according to the requirement of a given mission.
 *
 *
 */
class TransportNetwork : public Gecode::Space
{
    friend class templ::MissionPlanner;
public:
    typedef std::map<FluentTimeResource, organization_model::ModelPool > ModelDistribution;
    typedef std::map<FluentTimeResource, Role::List> RoleDistribution;

    class Solution
    {
        friend class TransportNetwork;

        symbols::constants::Location::PtrList mLocations;
        solvers::temporal::point_algebra::TimePoint::PtrList mTimepoints;


        ModelDistribution mModelDistribution;
        RoleDistribution mRoleDistribution;
        SpaceTime::Timelines mTimelines;
        SpaceTime::Network mMinCostFlowSolution;

    public:
        const ModelDistribution& getModelDistribution() const { return mModelDistribution; }
        const RoleDistribution& getRoleDistribution() const { return mRoleDistribution; }

        std::string toString(uint32_t indent = 0) const;
        SpaceTime::Network toNetwork() const;

        SpaceTime::Network getMinCostFlowSolution() const { return mMinCostFlowSolution; }
    };

    typedef std::vector<Solution> SolutionList;
    typedef shared_ptr<TransportNetwork> Ptr;
    typedef shared_ptr< Gecode::BAB<TransportNetwork> > BABSearchEnginePtr;

    const std::vector<solvers::temporal::Interval>& getIntervals() const { return mIntervals; }

    /**
     * Search state of the model distribution
     */
    class SearchState
    {
    public:
        enum Type { OPEN, SUCCESS, FAILED };

        SearchState(const Mission::Ptr& mission);

        SearchState(const TransportNetwork::Ptr& transportNetwork,
                const TransportNetwork::BABSearchEnginePtr& searchEngine = TransportNetwork::BABSearchEnginePtr());

        TransportNetwork::Ptr getInitialState() const { return mpInitialState; }
        Mission::Ptr getMission() const { return mpMission; }

        SearchState next() const;

        Type getType() const { return mType; }

        const ModelDistribution& getModelDistribution() const { return mSolution.getModelDistribution(); }
        const RoleDistribution& getRoleDistribution() const { return mSolution.getRoleDistribution(); }

        const Solution getSolution() const { return mSolution; }

    private:
        Mission::Ptr mpMission;
        TransportNetwork::Ptr mpInitialState;
        TransportNetwork::BABSearchEnginePtr mpSearchEngine;

        Type mType;
        Solution mSolution;
    };

    friend class SearchState;
private:
    /// The mission to plan for
    Mission::Ptr mpMission;

    /// The model pool -- in terms of available resources that can be used for
    /// planning
    organization_model::ModelPool mModelPool;
    /// The organization model
    organization_model::OrganizationModelAsk mAsk;

    /// All available services
    owlapi::model::IRIList mServices;
    /// All available resources
    owlapi::model::IRIList mResources;

    /// (Time)Intervals (as defined in the mission)
    std::vector<solvers::temporal::Interval> mIntervals;
    /// Timepoints
    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> mTimepoints;
    /// Constants: Locations (as defined in the mission)
    std::vector<symbols::constants::Location::Ptr> mLocations;

    /// List of FluentTimeResource which represents the functional
    /// requirements that arise from the mission scenario
    std::vector<FluentTimeResource> mResourceRequirements;

    // map timeslot to fluenttime service
    std::map<uint32_t, std::vector<FluentTimeResource> > mTimeIndexedRequirements;

    // ###########################
    // Model-based mapping
    // ###########################
    //
    // Array which will be accessed using Gecode::Matrix
    // The array contains information about the robot types required to
    // fullfill a requirement, e.g., r-0
    //
    // Additional resource constraints apply due to the upper resource
    // bound and time
    //  requirement | robot-type-0 | robot-type-1 | robot-type-2 | ...
    //  r-0         |      1       |      0       |      1       | ...
    //  r-1         |      0       |      2       |      2       | ...
    Gecode::IntVarArray mModelUsage;
    owlapi::model::IRIList mAvailableModels;

    // Per requirement: collect the extensional constraints for the models that
    // can fulfill the requirement
    std::map<uint32_t, Gecode::TupleSet> mExtensionalConstraints;


    // #######################
    // Role based mapping
    // #######################
    // Array which will be accessed using Gecode::Matrix
    // -- contains information about the robot types required to
    // fullfill r-0
    // Additional resource constraints apply due to the upper resource
    // bound and time
    //  requirement | role-0 | role-1 | role-2 | ...
    //  r-0         |   1    |   0    |   1    | ...
    //  r-1         |   0    |   1    |   1    | ...
    Gecode::IntVarArray mRoleUsage;

    // Constraints:
    // per role for overlapping fluents: sum <= 1 means they have to be distinct
    // per requirement/role: sum of same type roles <= model bound for fts
    //
    // model-based first stage guarantees conflict free solution on type basis
    Role::List mRoles;

    std::vector<uint32_t> mActiveRoles;
    Role::List mActiveRoleList;

    // ############################
    // Timelines
    // ############################
    // per role a timeline for each node in the path
    // in the form of an adjacency list for spatio-temporal tuples
    // (|Locations|*|Timepoints|)
    //
    // Activation if edge is traversed by this item or not
    ListOfAdjacencyLists mTimelines;

    std::vector<int32_t> mSupplyDemand;
    // Map the transport characteristic: (|Locations|*|Timepoints|)^2
    // Order such that bigger indexes are referring to later events(!)
    //                    | (t-0,loc-var-0) | (t-0, loc-var-1) | (t-0, loc-var-2) | ...
    // (t-0, loc-var-0)   |     0          |     0
    // (t-0, loc-var-1)   |     1          |     0
    // (t-0, loc-var-2)   |     1          |     0
    // (t-0, loc-trans)   |     1          |     0
    // (t-1, loc-var-0)   |
    std::vector< Gecode::IntVarArray > mTimelineGraphs;
    Gecode::IntVarArray mCapacities;
    //std::vector< Gecode::IntVarArray > mProviderCapacities;
    //std::vector< Gecode::IntVarArray > mConsumerCapacities;

    // row column access
    //MatrixXi mProviderCapacities;
    std::vector<transshipment::Flaw> mMinCostFlowFlaws;

private:

    std::set< std::vector<uint32_t> > toCSP(const organization_model::ModelPoolSet& set) const;
    std::vector<uint32_t> toCSP(const organization_model::ModelPool& combination) const;

    /**
     * Get the index of the system model in the csp representation
     * \return index of model
     */
    uint32_t systemModelToCSP(const owlapi::model::IRI& model) const;

    /**
     * Convert the given list of combinations to a Gecode::TupleSet, i.e. create
     * extensional constraints out of the combination set
     * \return TupleSet
     */
    void appendToTupleSet(Gecode::TupleSet& tupleSet, const organization_model::ModelPoolSet& combinations) const;

    /**
     * Map a fluents to its index (id)
     * \return index of the fluent
     */
    size_t getFluentIndex(const FluentTimeResource& fluent) const;
    /**
     * Map the resource model to its index (id)
     * \return index of the resource model
     */
    size_t getResourceModelIndex(const owlapi::model::IRI& model) const;
    const owlapi::model::IRI& getResourceModelFromIndex(size_t index) const;
    size_t getResourceModelMaxCardinality(size_t model) const;

    size_t getMaxResourceCount(const organization_model::ModelPool& model) const;

    /**
     * Check if given role (by id) is model (by id)
     * \return true if this is the case, false otherwise
     */
    bool isRoleForModel(uint32_t roleIndex, uint32_t modelIndex) const;

    /**
     * Get the set of roles that are actively used in the solutions
     * \return set of roles (by index id) that are active
     */
    std::vector<uint32_t> computeActiveRoles() const;

    static void assignRoles(Gecode::Space& home);
    /**
     *
     */
    void postRoleAssignments();

    static void minCostFlowAnalysis(Gecode::Space& home);
    void postMinCostFlowConstraints();

    static void generateTimelines(Gecode::Space& home);
    void postTimelines();

protected:
    // The general idea for implementing a LVNS approach
    //
    // a find a feasible/partial solution:
    //      since a fully feasible solution might not be found at all
    //      (a) which define the first instance of the problem, i.e. the required role distribution
    //      (b) use the flow optimization to find a first result
    //          - use the resulting violations in order to improve the actual result
    //            based on the following observations and constraint injections resolvers
    //            and continue the search --

    /**
     * \return true if restart is required, false otherwise
     */
    virtual bool master(const Gecode::MetaInfo& mi);

    /**
     * \return true if search is complete
     */
    virtual bool slave(const Gecode::MetaInfo& mi);

    /**
     * Initialize first for slave
     */
    void first();


    /**
     * Initalize slave for next solution
     */
    void next(const TransportNetwork& n);

    /**
     * Constraint this instance -- provide
     * a previous space to extract information for constrain
     */
    virtual void constrain(const Gecode::Space& n);


    /**
     * Get the solution of this Gecode::Space instance
     */
    Solution getSolution() const;

    ModelDistribution getModelDistribution() const;
    RoleDistribution getRoleDistribution() const;
    SpaceTime::Timelines getTimelines() const;

    uint32_t getTimepointIndex(const temporal::point_algebra::TimePoint::Ptr& timePoint) const;

    std::string toString(const std::vector<Gecode::IntVarArray>& timelines) const;

    /**
     * Set the cardinality constraints
     */
    void initializeMinMaxConstraints();

    /**
     * Identify overlapping constraints and set and upper resource bound
     */
    void setUpperBoundForConcurrentRequirements();

    /**
     * Require the number of active instances/roles to equal the number of required model instances
     *
     */
    void initializeRoleDistributionConstraints();

    /**
     * Limit the usage of instances/roles to 1 for concurrent requirements
     *
     * That guarantees that one role can only be use at a time
     */
    void enforceUnaryResourceUsage();

    Gecode::Symmetries identifySymmetries();

public:
    TransportNetwork(const templ::Mission::Ptr& mission);

    /**
     * Search support
     * This copy constructor is required for the search engine
     * and it has to provide a deep copy
     */
    TransportNetwork(bool share, TransportNetwork& s);

    /**
     * Creat a copy of this space
     * This method is called by the search engine
     */
    virtual Gecode::Space* copy(bool share);

    /**
     * Solve the mission
     */
    static SolutionList solve(const templ::Mission::Ptr& mission, uint32_t minNumberOfSolutions = 0);

    std::string toString() const;

    std::string modelUsageToString() const;
    std::string roleUsageToString() const;

    void print(std::ostream& os) const { os << toString() << std::endl; }

    /**
     * Add a particular function requirement with respect to a
     * FluentTimeResource instance
     */
    void addFunctionRequirement(const FluentTimeResource& fts, owlapi::model::IRI& function);

    size_t getNumberOfTimepoints() const { return mTimepoints.size(); }
    size_t getNumberOfFluents() const { return mLocations.size(); }
    std::vector<uint32_t> getActiveRoles() const { return mActiveRoles; }
    Role::List getActiveRoleList() const { return mActiveRoleList; }

};

std::ostream& operator<<(std::ostream& os, const TransportNetwork::Solution& solution);
std::ostream& operator<<(std::ostream& os, const TransportNetwork::SolutionList& solutions);

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP
