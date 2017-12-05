#ifndef TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP
#define TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP

#include <gecode/int.hh>
#include <set>
#include <organization_model/OrganizationModelAsk.hpp>
#include "../../Mission.hpp"
#include "../FluentTimeResource.hpp"

namespace templ {
namespace solvers {
namespace csp {

/**
 * General constraint that can be applied to mission
 */
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
     * \param home
     * \param roleUsage
     * \param requirements
     * \param fts0
     * \param fts1
     * \param roleModel
     * \param minDistinctRoles
     */
    static void minDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t minDistinctRoles);

    /**
     * Increment the distinction for model instances between two requirements
     * for \a additional
     */
    static void addDistinct(Gecode::Space& home,
            Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t additional);

    /**
     * Increment the distinction for model instances between two requirements
     * for \a additional
     *
     * add-model-distinction (fts0,fts1,roleModel,add)
     *
     */
    static void addDistinct(Gecode::Space& home, const Gecode::IntVarArray& roleUsageCurrent,
            Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t additional);

    static std::set<Role> getUniqueRoles(const Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel);

    /**
     * Add a model requirement (to one of the fluent time resource of the given
     * interval)
     */
    static void addModelRequirement(
            std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& ftr,
            const owlapi::model::IRI& model,
            uint32_t additional = 1);

    /**
     * Add a particular function requirement to the current mission with respect to a
     * FluentTimeResource instance
     * \throws std::invalid_argument when fluent time resource could not be
     * found
     */
    static void addFunctionRequirement(const owlapi::model::IRIList& allAvailableResources,
            std::vector<FluentTimeResource>& resourceRequirements,
            const FluentTimeResource& fts,
            const owlapi::model::IRI& function,
            organization_model::OrganizationModelAsk ask);

    /**
     * Add functionalities requirements (functionalities with particular property constraints)
     * with respect to a FluentTimeResource instance
     * \throws std::invalid_argument when fluent time resource could not be
     * found
     */
    static void addFunctionalitiesRequirement(const owlapi::model::IRIList& allAvailableResources,
            std::vector<FluentTimeResource>& resourceRequirements,
            const FluentTimeResource& fts,
            const organization_model::FunctionalityRequirement::Map& functionalitiesRequirements,
            organization_model::OrganizationModelAsk ask);

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP
