#ifndef TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP
#define TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP

#include <gecode/int.hh>
#include <set>
#include <organization_model/OrganizationModelAsk.hpp>
#include "../../Mission.hpp"
#include "../FluentTimeResource.hpp"
#include <numeric/Combinatorics.hpp>

namespace templ {
namespace solvers {
namespace csp {

/**
 * General constraint that can be applied to mission
 */
class MissionConstraints
{
public:
    // ###########################
    // MIN/MAX minimum usage of roles over multiple requirements
    // ###########################
    static void usage(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& allRoles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            Gecode::IntRelType relation,
            uint32_t use);

    static void min(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& allRoles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            uint32_t use);

    static void max(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& allRoles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            uint32_t use);

    // ###########################
    // ALL/MIN/MAX Distinct
    // ###########################
    /**
     * Enforce all model instances, i.e. roles, to be distinct for two requirements
     */
    static void allDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel);

    /**
     * Enforce all model instances, i.e. roles, to be distinct for a given set of requirements
     * So each role can only be available for one of the affected requirements
     */
    static void allDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& allRoles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel);

    /**
     * Require a minimum distinction of \a minDistinctRoles for model instance between two requirements
     * \param home
     * \param roleUsage
     * \param requirements
     * \param fts0
     * \param fts1
     * \param roleModel
     * \param minDistinctRoles
     */
    static void distinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t minMaxDistinctRoles,
            Gecode::IntRelType relation);

    static void distinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            uint32_t distinctRoles,
            Gecode::IntRelType relation);

    static void distinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, Gecode::IntVar& minMaxDistinctRoles,
            Gecode::IntRelType relation);

    static void distinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            Gecode::IntVar& minMaxDistinctRoles,
            Gecode::IntRelType relation);

    /**
     * Require a minimum distinction of \a minDistinctRoles for model instance between two requirements
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
     * Require a minimum distinction of \a minDistinctRoles for model instance between all paris fo multiple requirements
     * \param home
     * \param roleUsage
     * \param requirements
     * \param fts0
     * \param fts1
     * \param roleModel
     * \param minDistinctRoles
     * \see minDistinct
     */
    static void minDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            uint32_t minDistinctRoles);

    /**
     * Require a maximum distinction of \a minDistinctRoles for model instance between two requirements
     * \param home
     * \param roleUsage
     * \param requirements
     * \param fts0
     * \param fts1
     * \param roleModel
     */
    static void maxDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t maxDistinctRoles);

    /**
     * Require a maximum distinction of \a minDistinctRoles for model instance between two requirements
     * \param home
     * \param roleUsage
     * \param requirements
     * \param fts0
     * \param fts1
     * \param roleModel
     */
    static void maxDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, Gecode::IntVar& maxDistinctRoles);

    /**
     * Require a maximum distinction of \a distinctRoles for model instance between all paris fo multiple requirements
     * \param home
     * \param roleUsage
     * \param requirements
     * \param allRequirements
     * \param roleModel
     * \param distinctRoles
     * \see minDistinct
     */
    static void maxDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            uint32_t distinctRoles);

    static void maxDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            Gecode::IntVar& distinctRoles);

    /**
     * Extract the number of distinct roles
     */
    static uint32_t distinctRoles(Gecode::IntVarArray &roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource::List& ftrs,
            const owlapi::model::IRI& roleModel);

    // ###################
    // ALL/MIN/MAX EQUAL
    // ##################
    /**
     * Require a minimum distinction of \a minDistinctRoles for model instance between two requirements
     * \param home
     * \param roleUsage
     * \param requirements
     * \param fts0
     * \param fts1
     * \param roleModel
     * \param minDistinctRoles
     */
    static void equal(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t equalRoles,
            Gecode::IntRelType relation);

    static void equal(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            uint32_t equalRoles,
            Gecode::IntRelType relation);

    static void allEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel);

    static void allEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& allRoles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel);

    static void minEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t equalRoles);

    static void minEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            uint32_t equalRoles);

    static void maxEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
            const FluentTimeResource& fts0, const FluentTimeResource& fts1,
            const owlapi::model::IRI& roleModel, uint32_t equalRoles);

    static void maxEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
            const Role::List& roles,
            const FluentTimeResource::List& allRequirements,
            const FluentTimeResource::Set& affectedRequirements,
            const owlapi::model::IRI& roleModel,
            uint32_t equalRoles);

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

    static std::set<Role> getUniqueRoles(const Gecode::IntVarArray& roleUsage,
            const Role::List& roles, const std::vector<FluentTimeResource>& allRequirements,
            const std::vector<FluentTimeResource>& relevantRequirements,
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
     * Add a particular resource (function) requirement to the current mission with respect to a
     * FluentTimeResource instance
     * \throws std::invalid_argument when fluent time resource could not be
     * found
     */
    static void addResourceRequirement(const owlapi::model::IRIList& allAvailableResources,
            std::vector<FluentTimeResource>& resourceRequirements,
            const FluentTimeResource& fts,
            const organization_model::Resource& resource,
            organization_model::OrganizationModelAsk ask);

    /**
     * Add a particular resource (function) requirement to the current mission with respect
     * to a set of FluentTimeResource instances
     * \throw std::invalid_argument when fluent time resource could not be found
     */
    static void addResourceRequirement(const owlapi::model::IRIList& allAvailableResources,
            std::vector<FluentTimeResource>& resourceRequirements,
            const FluentTimeResource::Set& ftrs,
            const organization_model::Resource& resource,
            organization_model::OrganizationModelAsk ask);

private:
    static size_t getResourceIndex(const owlapi::model::IRIList& allAvailableResources,
            const organization_model::Resource& resource);
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_MISSION_CONSTRAINTS_HPP
