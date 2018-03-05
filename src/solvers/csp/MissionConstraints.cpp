#include "MissionConstraints.hpp"
#include <gecode/minimodel.hh>
#include "utils/Converter.hpp"
#include <organization_model/Algebra.hpp>

namespace templ {
namespace solvers {
namespace csp {

void MissionConstraints::usage(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& affectedRequirements,
        const owlapi::model::IRI& roleModel,
        Gecode::IntRelType relation,
        uint32_t use)
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ allRoles.size(), /*height --> row*/ allRequirements.size());

    Gecode::IntVarArgs rolesInvolvedArgs;
    // Each role/agent can only be available for maximum and exactly one single requirement
    for(size_t roleIndex = 0; roleIndex < allRoles.size(); ++roleIndex)
    {
        const Role& role = allRoles[roleIndex];
        if(role.getModel() != roleModel)
        {
            continue;
        }

        Gecode::IntVarArgs args;
        for(const FluentTimeResource& ftr : affectedRequirements)
        {
            size_t fluent = FluentTimeResource::getIndex(allRequirements, ftr);
            Gecode::IntVar roleActive = roleDistribution(roleIndex, fluent);
            args << roleActive;
        }

        Gecode::BoolVar roleInvolvement = Gecode::expr(home, sum(args) > 0);
        // Translate the bool var into a (countable) IntVar, so that we can sum
        // all roles that fulfill the 'full assignment' to the
        // affectedRequirements
        Gecode::IntVar rolePresent(home,0, allRequirements.size());
        channel(home, roleInvolvement, rolePresent);
        rolesInvolvedArgs << rolePresent;
    }

    // Apply constraints to the actual appearance of roles
    Gecode::IntVar sumOfFullAppearance = Gecode::expr(home, sum(rolesInvolvedArgs));
    rel(home, sumOfFullAppearance, relation, use);
}

void MissionConstraints::min(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& affectedRequirements,
        const owlapi::model::IRI& roleModel,
        uint32_t use)
{
    return usage(home, roleUsage,
            allRoles,
            allRequirements,
            affectedRequirements,
            roleModel,
            Gecode::IRT_GQ,
            use);
}

void MissionConstraints::max(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& affectedRequirements,
        const owlapi::model::IRI& roleModel,
        uint32_t use)

{
    return usage(home, roleUsage,
            allRoles,
            allRequirements,
            affectedRequirements,
            roleModel,
            Gecode::IRT_LQ,
            use);
}



void MissionConstraints::allDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel)
{
    FluentTimeResource::Set affectedRequirements = { fts0, fts1 };

    return allDistinct(home, roleUsage, roles, requirements, affectedRequirements, roleModel);
}

void MissionConstraints::allDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& affectedRequirements,
        const owlapi::model::IRI& roleModel)
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ allRoles.size(), /*height --> row*/ allRequirements.size());

    // Each role/agent can only be available for maximum and exactly one single requirement
    for(size_t roleIndex = 0; roleIndex < allRoles.size(); ++roleIndex)
    {
        const Role& role = allRoles[roleIndex];
        if(role.getModel() == roleModel)
        {
            Gecode::IntVarArgs args;
            for(const FluentTimeResource& ftr : affectedRequirements)
            {
                size_t fluent = FluentTimeResource::getIndex(allRequirements, ftr);
                Gecode::IntVar v = roleDistribution(roleIndex, fluent);
                args << v;
            }
            Gecode::rel(home, Gecode::sum(args) <= 1);
        }
    }
}

void MissionConstraints::distinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel, uint32_t distinctRoles,
        Gecode::IntRelType relation)
{
    std::vector<size_t> indices;
    for(size_t roleIndex = 0; roleIndex < roles.size(); ++roleIndex)
    {
        const Role& role = roles[roleIndex];
        if(role.getModel() == roleModel)
        {
            indices.push_back(roleIndex);
        }
    }

    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ roles.size(), /*height --> row*/ requirements.size());
    Gecode::IntVarArgs args;
    // For each role
    for(size_t m = 0; m < indices.size(); ++m)
    {
        size_t roleIndex = indices[m];
        size_t fluent0 = FluentTimeResource::getIndex(requirements, fts0);
        Gecode::IntVar v0 = roleDistribution(roleIndex, fluent0);

        size_t fluent1 = FluentTimeResource::getIndex(requirements, fts1);
        Gecode::IntVar v1 = roleDistribution(roleIndex, fluent1);

        // Check if a role is part of the fulfillment of both requirements
        // if so -- sum equals to 0 thus there is no distinction
        Gecode::IntVar rolePresentInBoth = Gecode::expr(home, abs(v0 - v1));
        args << rolePresentInBoth;
    }

    Gecode::IntVar sumOfAppearance = Gecode::expr(home, sum(args));
    rel(home, sumOfAppearance, relation, distinctRoles);
}

void MissionConstraints::distinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& _affectedRequirements,
        const owlapi::model::IRI& roleModel,
        uint32_t distinctRoles,
        Gecode::IntRelType relation)
{
    FluentTimeResource::List affectedRequirements(_affectedRequirements.begin(), _affectedRequirements.end());

    for(size_t a = 0; a < affectedRequirements.size() - 1; ++a)
    {
        for(size_t b = a+1; b < affectedRequirements.size(); ++b)
        {
            distinct(home, roleUsage, allRoles, allRequirements,
                    affectedRequirements[a],
                    affectedRequirements[b],
                    roleModel,
                    distinctRoles,
                    relation);
        }
    }
}

void MissionConstraints::minDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel, uint32_t minDistinctRoles)
{
    return distinct(home, roleUsage, roles, requirements,
            fts0, fts1,
            roleModel, minDistinctRoles,
            Gecode::IRT_GQ);
}

void MissionConstraints::maxDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel, uint32_t maxDistinctRoles)
{
    return distinct(home, roleUsage, roles, requirements,
            fts0, fts1,
            roleModel, maxDistinctRoles,
            Gecode::IRT_LQ);
}

void MissionConstraints::minDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& affectedRequirements,
        const owlapi::model::IRI& roleModel,
        uint32_t distinctRoles)
{
    return distinct(home, roleUsage, allRoles,
            allRequirements, affectedRequirements,
            roleModel,
            distinctRoles,
            Gecode::IRT_GQ);
}

void MissionConstraints::maxDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& affectedRequirements,
        const owlapi::model::IRI& roleModel,
        uint32_t distinctRoles)
{
    return distinct(home, roleUsage, allRoles,
            allRequirements, affectedRequirements,
            roleModel,
            distinctRoles,
            Gecode::IRT_LQ);
}

void MissionConstraints::equal(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel, uint32_t equalRoles,
        Gecode::IntRelType relation)
{
    FluentTimeResource::Set affectedRequirements = { fts0, fts1 };

    return equal(home, roleUsage, allRoles,
            requirements,
            affectedRequirements,
            roleModel,
            equalRoles,
            relation);
}

void MissionConstraints::equal(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& affectedRequirements,
        const owlapi::model::IRI& roleModel,
        uint32_t equalRoles,
        Gecode::IntRelType relation)
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ allRoles.size(), /*height --> row*/ allRequirements.size());

    Gecode::IntVarArgs rolesInvolvedArgs;
    // Each role/agent can only be available for maximum and exactly one single requirement
    for(size_t roleIndex = 0; roleIndex < allRoles.size(); ++roleIndex)
    {
        const Role& role = allRoles[roleIndex];
        if(role.getModel() != roleModel)
        {
            continue;
        }

        Gecode::IntVarArgs args;
        for(const FluentTimeResource& ftr : affectedRequirements)
        {
            size_t fluent = FluentTimeResource::getIndex(allRequirements, ftr);
            Gecode::IntVar roleActive = roleDistribution(roleIndex, fluent);
            args << roleActive;
        }

        // Get the count of a roles involvement and check whether it is involved
        // in all affected requirements
        Gecode::BoolVar rolePresentInAll = Gecode::expr(home, affectedRequirements.size() == sum(args));

        // Translate the bool var into a (countable) IntVar, so that we can sum
        // all roles that fulfill the 'full assignment' to the
        // affectedRequirements
        Gecode::IntVar rolePresent(home, 0, affectedRequirements.size());
        channel(home, rolePresentInAll, rolePresent);

        rolesInvolvedArgs << rolePresent;
    }

    // Apply constraints to the full assignment
    Gecode::IntVar sumOfFullAppearance = Gecode::expr(home, sum(rolesInvolvedArgs));
    rel(home, sumOfFullAppearance, relation, equalRoles);
}

void MissionConstraints::allEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel)
{
    return maxDistinct(home, roleUsage, roles,
            requirements,
            fts0, fts1,
            roleModel, 0);
}

void MissionConstraints::allEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& allRoles,
        const FluentTimeResource::List& allRequirements,
        const FluentTimeResource::Set& affectedRequirements,
        const owlapi::model::IRI& roleModel)
{
    return maxDistinct(home, roleUsage, allRoles,
            allRequirements,
            affectedRequirements,
            roleModel, 0);
}


void MissionConstraints::minEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
 const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
 const FluentTimeResource& fts0, const FluentTimeResource& fts1,
 const owlapi::model::IRI& roleModel, uint32_t equalRoles)
{
    return equal(home, roleUsage, roles,
            requirements,
            fts0, fts1,
            roleModel,
            equalRoles,
            Gecode::IRT_GQ);

}

void MissionConstraints::minEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
 const Role::List& roles,
 const FluentTimeResource::List& allRequirements,
 const FluentTimeResource::Set& affectedRequirements,
 const owlapi::model::IRI& roleModel,
 uint32_t equalRoles)
{
    return equal(home, roleUsage, roles,
            allRequirements,
            affectedRequirements,
            roleModel,
            equalRoles,
            Gecode::IRT_GQ);

}

void MissionConstraints::maxEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
 const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
 const FluentTimeResource& fts0, const FluentTimeResource& fts1,
 const owlapi::model::IRI& roleModel, uint32_t equalRoles)
{
    return equal(home, roleUsage, roles,
            requirements,
            fts0, fts1,
            roleModel,
            equalRoles,
            Gecode::IRT_LQ);
}

void MissionConstraints::maxEqual(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
 const Role::List& roles,
 const FluentTimeResource::List& allRequirements,
 const FluentTimeResource::Set& affectedRequirements,
 const owlapi::model::IRI& roleModel,
 uint32_t equalRoles)
{
    return equal(home, roleUsage, roles,
            allRequirements,
            affectedRequirements,
            roleModel,
            equalRoles,
            Gecode::IRT_LQ);
}

void MissionConstraints::addDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel, uint32_t additional)
{
    // Adding this constraint will only work to an already once solved instance
    // of the problem
    std::set<Role> uniqueRoles = getUniqueRoles(roleUsage, roles, requirements,
            fts0, fts1,
            roleModel);
    size_t numberOfUniqueRoles = uniqueRoles.size();
    LOG_WARN_S << "Previous number of unique roles: " << numberOfUniqueRoles << " -- should be increased with " << additional;
    minDistinct(home, roleUsage, roles, requirements, fts0, fts1, roleModel, numberOfUniqueRoles + additional);
}

void MissionConstraints::addDistinct(Gecode::Space& home,
        const Gecode::IntVarArray& roleUsageCurrent, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel, uint32_t additional)
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ roles.size(), /*height --> row*/ requirements.size());

    // Adding this constraint will only work to an already once solved instance
    // of the problem
    std::set<Role> uniqueRoles = getUniqueRoles(roleUsageCurrent, roles, requirements,
            fts0, fts1,
            roleModel);
    size_t numberOfUniqueRoles = uniqueRoles.size();
    LOG_WARN_S << "Previous number of unique roles: " << numberOfUniqueRoles << " -- should be increased with " << additional;
    minDistinct(home, roleUsage, roles, requirements, fts0, fts1, roleModel, numberOfUniqueRoles + additional);
}

std::set<Role> MissionConstraints::getUniqueRoles(const Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel)
{
    FluentTimeResource::List ftrs;
    ftrs.push_back(fts0);
    ftrs.push_back(fts1);

    return getUniqueRoles(roleUsage, roles, requirements,
            ftrs, roleModel);
}

std::set<Role> MissionConstraints::getUniqueRoles(const Gecode::IntVarArray& constRoleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& allRequirements,
        const std::vector<FluentTimeResource>& relevantRequirements,
        const owlapi::model::IRI& roleModel)
{
    Gecode::IntVarArray& roleUsage = const_cast<Gecode::IntVarArray&>(constRoleUsage);
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ roles.size(), /*height --> row*/ allRequirements.size());

    std::set<Role> assignedRoles;
    for(size_t r = 0; r < roles.size(); ++r)
    {
        const Role& role = roles[r];
        if(role.getModel() != roleModel)
        {
            continue;
        }

        for(const FluentTimeResource& ftr : relevantRequirements)
        {
            size_t requirementIdx = FluentTimeResource::getIndex(allRequirements, ftr);

            const Gecode::IntVar& var = roleDistribution(r, requirementIdx);
            if(!var.assigned())
            {
                throw std::runtime_error("templ::solvers::csp::MissionConstraints::getUniqueRoles: can only be performed on a fully assigned array. Failed for role: '" + roles[r].toString() + "'");
            }

            Gecode::IntVarValues v( var );
            if( v.val() == 1 )
            {
                assignedRoles.insert( role );
                // continue with next role -- one has been found to be involved
                break;
            }
        }
    }
    return assignedRoles;
}

void MissionConstraints::addModelRequirement(
        std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& ftr,
        const owlapi::model::IRI& model,
        uint32_t additional)
{
    std::vector<FluentTimeResource>::iterator it = requirements.begin();
    for(; it != requirements.end(); ++it)
    {
        if(*it == ftr)
        {
            // add function requirement --> check addResourceRequirement
            // update FTR modelpool
            // (add max cardinalities)
            std::cout << "Updated min cardinalities: for " << model.toString() << std::endl << it->toString(4);
            return;
        }
    }
    throw std::invalid_argument("templ::solvers::csp::MissionConstraints::addModelRequirement: given FluentTimeResource is not part of the given list");
}

void MissionConstraints::addResourceRequirement(const owlapi::model::IRIList& allAvailableResources,
        std::vector<FluentTimeResource>& resourceRequirements,
        const FluentTimeResource& fts,
        const organization_model::Resource& resource,
        organization_model::OrganizationModelAsk ask)
{
    // Find the function requirement index
    size_t index = getResourceIndex(allAvailableResources, resource);
    // identify the fluent time resource
    size_t idx = FluentTimeResource::getIndex(resourceRequirements, fts);
    FluentTimeResource& ftr = resourceRequirements[idx];

    LOG_DEBUG_S << "Fluent before adding function requirement: " << ftr.toString();

    ftr.addResourceIdx(index);
    ftr.addRequiredResource(resource);
    ftr.updateSatisficingCardinalities();

    // TODO: should max be really changed here, it should be rather min to be
    // updated -- though it can be checked here if max cardinalities are not
    // exceeded
    ftr.setMaxCardinalities( organization_model::Algebra::max(ftr.getMaxCardinalities(), ask.getFunctionalSaturationBound(resource) ));
    LOG_DEBUG_S << "Fluent after adding function requirement: " << ftr.toString();
}

void MissionConstraints::addResourceRequirement(const owlapi::model::IRIList& allAvailableResources,
        std::vector<FluentTimeResource>& resourceRequirements,
        const FluentTimeResource::Set& ftrs,
        const organization_model::Resource& resource,
        organization_model::OrganizationModelAsk ask)
{
    for(const FluentTimeResource& ftr : ftrs)
    {
        addResourceRequirement(allAvailableResources,
                resourceRequirements,
                ftr,
                resource,
                ask);
    }
}

size_t MissionConstraints::getResourceIndex(const owlapi::model::IRIList& allAvailableResources,
        const organization_model::Resource& resource)
{
    // Find the function requirement index
    size_t index = 0;
    owlapi::model::IRIList::const_iterator cit = allAvailableResources.begin();
    for(; cit != allAvailableResources.end(); ++cit, ++index)
    {
        if(*cit == resource.getModel())
        {
            break;
        }
    }

    // If function cannot be found add the function to the (known) required resources
    if(index >= allAvailableResources.size())
    {
            throw std::invalid_argument("templ::solvers::csp::MissionConstraints::getResourceIndex: could not find the resource index for: '" + resource.getModel().toString() + "'");
    }

    return index;
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ

