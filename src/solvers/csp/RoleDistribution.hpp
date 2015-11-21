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
    owlapi::model::IRIList mAvailableModels;

public:
    typedef std::map<FluentTimeResource, Role::List> Solution;
    typedef std::vector<Solution> SolutionList;

    /**
     * Default constructor
     */
    RoleDistribution(const Mission& mission, const ModelDistribution::Solution& modelDistribution); 

    /**
     * Search support
     * This copy constructor is required for the search engine
     * and it has to provide a deep copy
     */
    RoleDistribution(bool share, RoleDistribution& other);

    Gecode::Space* copy(bool share);

    /**
     * Get the solution of this Gecode::Space instance
     */
    Solution getSolution() const;


    std::string toString() const;
    void print(std::ostream& os) const { os << toString() << std::endl; }

    static SolutionList solve(const Mission& mission, const ModelDistribution::Solution& modelDistribution);

    RoleDistribution* nextSolution();

    size_t getFluentIndex(const FluentTimeResource& fluent) const;

    const solvers::temporal::Interval& getInterval(size_t index) const { return mIntervals.at(index); }

    /**
     * Enforce a role to be distinct for two requirements
     */
    void distinct(const FluentTimeResource& fts0, const FluentTimeResource& fts1, const owlapi::model::IRI& model);
};

std::ostream& operator<<(std::ostream& os, const RoleDistribution::Solution& solution);
std::ostream& operator<<(std::ostream& os, const RoleDistribution::SolutionList& solutions);

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_ROLE_DISTRIBUTION_HPP
