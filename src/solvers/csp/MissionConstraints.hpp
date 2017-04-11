#ifndef TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP
#define TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP

#include "FluentTimeResource.hpp"
#include <gecode/int.hh>
#include <set>

namespace templ {
namespace solvers {
namespace csp {

class MissionConstraints
{
public:
    /**
     * Enforce all model instances, i.e. roles, to be distinct for two requirements
     */
    static void allDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel);

    /**
     * Require a minimum distinction of \a minDistinctRoles for model instance betwenn two requirements
     */
    static void minDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t minDistinctRoles);

    /**
     * Increment the distinction for model instances between two requirements
     * for \a additional
     */
    static void addDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t additional);

    static std::set<Role> getUniqueRoles(Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel);

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP
