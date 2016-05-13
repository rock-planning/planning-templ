#ifndef TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP
#define TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP

#include <string>
#include <map>
#include <vector>
#include <gecode/set.hh>
#include <gecode/search.hh>

#include <organization_model/OrganizationModelAsk.hpp>

#include <templ/Mission.hpp>
#include <templ/solvers/csp/FluentTimeResource.hpp>

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

        ModelDistribution mModelDistribution;
        RoleDistribution mRoleDistribution;
    public:
        const ModelDistribution& getModelDistribution() const { return mModelDistribution; }
        const RoleDistribution& getRoleDistribution() const { return mRoleDistribution; }

        std::string toString(uint32_t indent = 0) const;
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

    /// Service Requirement that arise from the mission scenario
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
    // ############################
    // Timelines
    // ############################
    // per role a timeline for each node in the path
    // in the form of an adjacency list for spatio-temporal tuples
    // (|Locations|*|Timepoints|)^2
    //
    // Activation if edge is traversed by this item or not
    std::vector< Gecode::IntVarArray > mTimelines;

    std::vector< Gecode::IntVarArray > mTimelineGraphs;

    // Map the transport characteristic: (|Locations|*|Timepoints|)^2
    // Order such that bigger indexes are referring to later events(!)
    //                    | (t-0,loc-var-0) | (t-0, loc-var-1) | (t-0, loc-var-2) | ...
    // (t-0, loc-var-0)   |     0          |     0
    // (t-0, loc-var-1)   |     1          |     0
    // (t-0, loc-var-2)   |     1          |     0
    // (t-1, loc-var-0)   |
    std::vector< Gecode::IntVarArray > mProvidedCapacities;
    std::vector< Gecode::IntVarArray > mConsumedCapacities;

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

    size_t getFluentIndex(const FluentTimeResource& fluent) const;
    size_t getResourceModelIndex(const owlapi::model::IRI& model) const;
    const owlapi::model::IRI& getResourceModelFromIndex(size_t index) const;
    size_t getResourceModelMaxCardinality(size_t model) const;

    std::vector<FluentTimeResource> getResourceRequirements() const;

    size_t getMaxResourceCount(const organization_model::ModelPool& model) const;

    /**
     * Create a compact representation for all requirement that
     * refer to the same fluent and time
     */
    void compact(std::vector<FluentTimeResource>& requirements) const;

    /**
     * Check if given role (by id) is model (by id)
     * \return true if this is the case, false otherwise
     */
    bool isRoleForModel(uint32_t roleIndex, uint32_t modelIndex) const;

    static void postRoleAssignments(Gecode::Space& home);
    void postRoleAssignments();

    static void postRoleTimelines(Gecode::Space& home);
    void postRoleTimelines();

protected:
    /**
     * Get the solution of this Gecode::Space instance
     */
    Solution getSolution() const;

    ModelDistribution getModelDistribution() const;
    RoleDistribution getRoleDistribution() const;

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

    static SolutionList solve(const templ::Mission::Ptr& mission);

    std::string toString() const;
    void print(std::ostream& os) const { os << toString() << std::endl; }

    void addFunctionRequirement(const FluentTimeResource& fts, owlapi::model::IRI& function);
};

std::ostream& operator<<(std::ostream& os, const TransportNetwork::Solution& solution);
std::ostream& operator<<(std::ostream& os, const TransportNetwork::SolutionList& solutions);

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TRANSPORT_NETWORK_HPP
