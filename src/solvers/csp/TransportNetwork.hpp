#ifndef TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP
#define TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP

#include <string>
#include <map>
#include <vector>
#include <gecode/set.hh>
#include <gecode/search.hh>

#include <moreorg/OrganizationModelAsk.hpp>
#include <qxcfg/Configuration.hpp>

#include "../../Mission.hpp"
#include "../FluentTimeResource.hpp"
#include "../Solver.hpp"
#include "../transshipment/MinCostFlow.hpp"
#include "FlawResolution.hpp"
#include "TemporalConstraintNetwork.hpp"
#include "Types.hpp"
#include "utils/FluentTimeIndex.hpp"
#include "Context.hpp"
#include "../SolutionAnalysis.hpp"
#include <moreorg/ResourceInstance.hpp>

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
class TransportNetwork : public Gecode::Space, public Solver
{
    friend class templ::MissionPlanner;
    friend class templ::solvers::Solver;
    friend class FlawResolution;
    friend class MissionConstraintManager;

public:
    typedef std::map<FluentTimeResource, moreorg::ModelPool > ModelDistribution;
    typedef std::map<FluentTimeResource, Role::List> RoleDistribution;

    /**
     * Solution for a TransportNetwork problem
     */
    class Solution
    {
        friend class TransportNetwork;

        symbols::constants::Location::PtrList mLocations;
        solvers::temporal::point_algebra::TimePoint::PtrList mTimepoints;


        ModelDistribution mModelDistribution;
        RoleDistribution mRoleDistribution;
        std::map<Role, csp::RoleTimeline> mTimelines;
        SpaceTime::Network mMinCostFlowSolution;
        SolutionAnalysis mSolutionAnalysis;

    public:
        const ModelDistribution& getModelDistribution() const { return mModelDistribution; }
        const RoleDistribution& getRoleDistribution() const { return mRoleDistribution; }

        std::string toString(uint32_t indent = 0) const;
        SpaceTime::Network toNetwork() const;

        SpaceTime::Network getMinCostFlowSolution() const { return mMinCostFlowSolution; }

        /**
         * Get the solution analysis that corresponds to this
         * solution
         */
        const SolutionAnalysis& getSolutionAnalysis() const  { return mSolutionAnalysis; }
    };

    typedef std::vector<Solution> SolutionList;
    typedef shared_ptr<TransportNetwork> Ptr;
    typedef shared_ptr< Gecode::BAB<TransportNetwork> > BABSearchEnginePtr;

    const std::vector<solvers::temporal::Interval>& getIntervals() const { return mpContext->intervals(); }

protected:
    /// The mission to plan for
    Mission::Ptr mpMission;
    Context::Ptr mpContext;

    /// Timepoints (will be sorted after postTemporalConstraints has been
    /// called)
    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> mTimepoints;

    /// List of FluentTimeResource which represents the functional
    /// requirements that arise from the mission scenario
    std::vector<FluentTimeResource> mResourceRequirements;

    /// map timeslot to fluenttime service
    std::map<uint32_t, std::vector<FluentTimeResource> > mTimeIndexedRequirements;

    /// ###############################
    /// Temporal constraint networks
    /// ###############################
    /// Solver/helper instance for the qualitative TCN
    TemporalConstraintNetworkBase mTemporalConstraintNetwork;
    // Gecode representation of the qualitative TCN
    Gecode::IntVarArray mQualitativeTimepoints;
    // The currently considered qualitative TCN without gaps
    temporal::QualitativeTemporalConstraintNetwork::Ptr mpQualitativeTemporalConstraintNetwork;

    /// ###########################
    /// Model-based mapping
    /// ###########################
    ///
    /// Array which will be accessed using Gecode::Matrix
    /// The array contains information about the robot types required to
    /// fullfill a requirement, e.g., r-0
    ///
    /// Additional resource constraints apply due to the upper resource
    /// bound and time
    ///  requirement | robot-type-0 | robot-type-1 | robot-type-2 | ...
    ///  r-0         |      1       |      0       |      1       | ...
    ///  r-1         |      0       |      2       |      2       | ...
    Gecode::IntVarArray mModelUsage;

    /// #######################
    /// Role based mapping
    /// #######################
    /// Array which will be accessed using Gecode::Matrix
    /// -- contains information about the robot types required to
    /// fullfill r-0
    /// Additional resource constraints apply due to the upper resource
    /// bound and time
    ///  requirement | role-0 | role-1 | role-2 | ...
    ///  r-0         |   1    |   0    |   1    | ...
    ///  r-1         |   0    |   1    |   1    | ...
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
    std::map<Role, csp::RoleTimeline> mMinRequiredTimelines;

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
    Gecode::IntVar mCost;
    Gecode::IntVar mNumberOfFlaws;

    // row column access
    //MatrixXi mProviderCapacities;
    SpaceTime::Network mMinCostFlowSolution;
    std::vector<transshipment::Flaw> mMinCostFlowFlaws;
    FlawResolution mFlawResolution;
    FlawResolution::ResolutionOptions mRequiredResolutionOptions;

    /// Flag to control the interactive mode
    static bool msInteractive;

    bool mUseMasterSlave;
    // The current master space
    TransportNetwork* mpCurrentMaster;

    typedef std::pair< std::vector<transshipment::Flaw>, SpaceTime::Network> FlowSolutionValue;
    typedef std::pair< std::map<Role, csp::RoleTimeline>, std::map<Role, csp::RoleTimeline> > FlowSolutionKey;
    typedef std::map< FlowSolutionKey, FlowSolutionValue > FlowSolutions;

    static FlowSolutions msMinCostFlowSolutions;

    /// List of extra constraints
    Constraint::PtrList mConstraints;
    SolutionAnalysis mSolutionAnalysis;

private:
    std::stringstream mInteractiveMessageStream;

    std::set< std::vector<uint32_t> > toCSP(const moreorg::ModelPool::Set& set) const;
    std::vector<uint32_t> toCSP(const moreorg::ModelPool& combination) const;

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
    void appendToTupleSet(Gecode::TupleSet& tupleSet, const moreorg::ModelPool::Set& combinations) const;

    /**
     * Map the resource model to its index (id)
     * \return index of the resource model
     */
    size_t getResourceModelIndex(const owlapi::model::IRI& model) const;
    const owlapi::model::IRI& getResourceModelFromIndex(size_t index) const;
    size_t getResourceModelMaxCardinality(size_t model) const;

    /**
     * Check if given role (by id) is model (by id)
     * \return true if this is the case, false otherwise
     */
    bool isRoleForModel(uint32_t roleIndex, uint32_t modelIndex) const;

    /**
     * Get the set of roles that are actively used in the solutions
     * \todo depending on the solution heuristic it should be possible to
     * set
     * \return set of roles (by index id) that are active
     */
    std::vector<uint32_t> computeActiveRoles() const;

    /**
     * Get the current (minimum) model assignment for a FluentTimeResource
     */
    moreorg::ModelPool currentMinModelAssignment(const FluentTimeResource& ftr) const;

    static void doPostTemporalConstraints(Gecode::Space& home);
    void postTemporalConstraints();

    static void doPostMinMaxConstraints(Gecode::Space& home);
    static void doPostExtensionalConstraints(Gecode::Space& home);

    static void doPostRoleAssignments(Gecode::Space& home);
    void postRoleAssignments();

    static void doPostMinCostFlow(Gecode::Space& home);
    void postMinCostFlow();

    static void doPostTimelines(Gecode::Space& home);
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
     * Initalize slave for next solution
     */
    void next(const TransportNetwork& n, const Gecode::MetaInfo& mi);

    /**
     * Constraint this instance -- provide
     * a previous space to extract information for constrain
     */
    virtual void constrain(const Gecode::Space& n);

    /**
     * Constraint this instance -- provide
     * a previous space to extract information for constrain
     */
    virtual void constrainSlave(const Gecode::Space& n);


    ModelDistribution getModelDistribution() const;
    RoleDistribution getRoleDistribution() const;

    /**
     * Use the list of active role in order to compute a list of timelines
     */
    std::map<Role, csp::RoleTimeline> getTimelines() const;

    /**
     * Identify the index of the given timepoint
     */
    uint32_t getTimepointIndex(const temporal::point_algebra::TimePoint::Ptr& timePoint) const;

    /**
     * Initialize the temporal constraint network and active the branching
     */
    void initializeTemporalConstraintNetwork();

    /**
     * Set the cardinality constraints as defined in the
     * mission specification
     */
    void initializeMinMaxConstraints();

    /**
     * Compute the extensional constraints, i.e.
     * allowed composite agents
     */
    void addExtensionalConstraints();

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
     * Apply all extra mission constraints that are part of the original
     * mission specification
     */
    void applyMissionConstraints();

    /**
     * Apply all 'extra' constraints
     */
    void applyExtraConstraints();

    /**
     * Apply access constraints
     */
    void applyAccessConstraints(ListOfAdjacencyLists& timelines,
            size_t numberOfTimepoints,
            size_t numberOfLocations,
            const Role::List& roles);

    /**
     * Limit the usage of instances/roles to 1 for concurrent requirements
     *
     * That guarantees that one role can only be use at a time
     */
    void enforceUnaryResourceUsage();

    Gecode::Symmetries identifySymmetries();

    Gecode::IntVar cost(void) const { return mCost; }

    void setUseMasterSlave(bool v) { mUseMasterSlave = v; }

public:
    TransportNetwork();

    /**
     * Construct CSP-based solver for a particular mission
     */
    TransportNetwork(const templ::Mission::Ptr& mission, const qxcfg::Configuration& configuration = qxcfg::Configuration());

    /**
     * Search support
     * This copy constructor is required for the search engine
     * and it has to provide a deep copy
     */
    TransportNetwork(TransportNetwork& s);

    /**
     * Creat a copy of this space
     * This method is called by the search engine
     */
    virtual Gecode::Space* copy(void);

    /**
     * Solve the mission
     * \param mission The mission specification to solve
     * \param minNumberOfSolutions Minimum number of solutions: -1 to run until
     *  a timeout happens, 0, to stop after first iteration and, > 0 to stop
     *  after given number of solutions has been found
     * \param configuration Configuration for this planning instance
     */
    static SolutionList solve(const templ::Mission::Ptr& mission, uint32_t minNumberOfSolutions = 1, const qxcfg::Configuration& configuration = qxcfg::Configuration(), const moreorg::ResourceInstance::List& componentBlacklist = moreorg::ResourceInstance::List());

    /**
     * Get the solution of this Gecode::Space instance
     */
    Solution getSolution() const;

    /**
     * Save a found solution
     */
    static void saveSolution(const Solution& solution, const Mission::Ptr& mission);

    solvers::Session::Ptr run(const templ::Mission::Ptr& mission, uint32_t minNumberOfSolutions, const qxcfg::Configuration& configuration);

    /**
     * Add a general constraint
     */
    void addConstraint(const Constraint::Ptr& constraint, TransportNetwork& network);

    /**
     * Get the current assignments as constraints
     */
    Constraint::PtrList getAssignmentsAsConstraints() const;

    /**
     * Create a mission that is augmented with the current assignments encoded
     * into constraints
     *
     * This allows to create a new mission specification out of an existing
     * assignment of agent roles
     * \return Mission
     */
    Mission::Ptr getAugmentedMission() const;

    /**
     * Propagate the solution of the location search for validation
     */
    Gecode::ExecStatus propagateImmobileAgentConstraints(const SpaceTime::Network& network);

    Gecode::ExecStatus updateTimeline(size_t timelineIdx,
            size_t locationIdx,
            size_t timepointIdx,
            size_t numberOfLocations,
            size_t numberOfTimepoints);

    std::string toString() const;

    std::string toString(const std::vector<Gecode::IntVarArray>& timelines) const;

    std::string modelUsageToString() const;
    std::string roleUsageToString() const;

    void print(std::ostream& os) const { os << toString() << std::endl; }

    /**
     * Save the output of toString() into a file
     * default is saved in the session folder as 'transport-network.status'
     */
    void save(const std::string& filename = "") const;

    size_t getNumberOfTimepoints() const { return mpContext->getNumberOfTimepoints(); }
    size_t getNumberOfFluents() const { return mpContext->getNumberOfFluents(); }
    /**
     * Get the active roles (as index list)
     */
    std::vector<uint32_t> getActiveRoles() const { return mActiveRoles; }

    /**
     * Get the list of active role (as role list)
     */
    Role::List getActiveRoleList() const { return mActiveRoleList; }

    void setCurrentMaster(TransportNetwork* master) { mpCurrentMaster = master; }

    static void breakpoint(const std::string& msg);

    /**
     * Start a breakpoint for the interactive mode
     */
    std::ostream& breakpointStart();

    /**
     * Close the interactive mode
     */
    void breakpointEnd();
};

std::ostream& operator<<(std::ostream& os, const TransportNetwork::Solution& solution);
std::ostream& operator<<(std::ostream& os, const TransportNetwork::SolutionList& solutions);

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP
