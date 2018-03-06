#ifndef TEMPL_SOLVERS_CSP_ROLE_DISTRIBUTION_HPP
#define TEMPL_SOLVERS_CSP_ROLE_DISTRIBUTION_HPP

#include <templ/solvers/csp/ModelDistribution.hpp>

namespace templ {
namespace solvers {
namespace csp {

class RoleDistribution : public Gecode::Space
{

    // Array which will be access using Gecode::Matrix
    // -- contains information about the robot types required to
    // fullfill r-0
    // Additional resource constraints apply due to the upper resource
    // bound and time
    //  requirement | role-0 | role-1 | role-2 | ...
    //  r-0         |   1    |   0    |   1    | ...
    //  r-1         |   0    |   1    |   1    | ...
    Gecode::IntVarArray mRoleUsage;

    // Constraints:
    // per role for overlapping fluents: sum <= 1 -- distinct
    // per requirement/role: sum of same type roles <= model bound for fts
    //
    // model-based first stage guarantees conflict free solution on type basis

    Role::List mRoles;
    FluentTimeResource::List mRequirements;

    // Get time intervals
    std::vector<solvers::temporal::Interval> mIntervals;

public:
    typedef std::map<FluentTimeResource, Role::List> Solution;
    typedef std::vector<Solution> SolutionList;

    typedef shared_ptr<RoleDistribution> Ptr;
    typedef shared_ptr< Gecode::BAB<RoleDistribution> > BABSearchEnginePtr;

    /**
     * Search state for the role distribution
     * to allow for incremental construction of solutions
     */
    class SearchState
    {
    public:
        enum Type { OPEN, SUCCESS, FAILED };

        SearchState(const ModelDistribution::SearchState& modelSearchState);

        SearchState(const RoleDistribution::Ptr& modelDistribution,
                const RoleDistribution::BABSearchEnginePtr& searchEngine = RoleDistribution::BABSearchEnginePtr());

        RoleDistribution::Ptr getInitialState() const { return mpInitialState; }

        SearchState next() const;

        Type getType() const { return mType; }
        const Solution& getSolution() const { return mSolution; }

    private:
        RoleDistribution::Ptr mpInitialState;
        RoleDistribution::BABSearchEnginePtr mpSearchEngine;

        Type mType;
        Solution mSolution;
    };

    friend class SearchState;

    /**
     * Default constructor
     */
    RoleDistribution(const Mission::Ptr& mission, const ModelDistribution::ModelDistributionSolution& modelDistribution);

    /**
     * Search support
     * This copy constructor is required for the search engine
     * and it has to provide a deep copy
     */
    RoleDistribution(RoleDistribution& other);

    Gecode::Space* copy();

    /**
     * Get the solution of this Gecode::Space instance
     */
    Solution getSolution() const;


    std::string toString() const;
    void print(std::ostream& os) const { os << toString() << std::endl; }

    static SolutionList solve(const Mission::Ptr& mission, const ModelDistribution::ModelDistributionSolution& modelDistribution);

    size_t getFluentIndex(const FluentTimeResource& fluent) const;

    const solvers::temporal::Interval& getInterval(size_t index) const { return mIntervals.at(index); }

    /**
     * Enforce all model instances to be distinct for two requirements
     */
    void allDistinct(const FluentTimeResource& fts0, const FluentTimeResource& fts1, const owlapi::model::IRI& model);
    void minDistinct(const FluentTimeResource& fts0, const FluentTimeResource& fts1, const owlapi::model::IRI& model, uint32_t minDistinctRoles);

    /**
     * Add an additional level of distinction for this type of role based on an
     * existing solution
     */
    void addDistinct(const FluentTimeResource& fts0, const FluentTimeResource& fts1, const owlapi::model::IRI& model, uint32_t additional, const Solution& solution);
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ

namespace std {
    std::ostream& operator<<(std::ostream& os, const templ::solvers::csp::RoleDistribution::Solution& solution);
    std::ostream& operator<<(std::ostream& os, const templ::solvers::csp::RoleDistribution::SolutionList& solutions);
}

#endif // TEMPL_SOLVERS_CSP_ROLE_DISTRIBUTION_HPP
