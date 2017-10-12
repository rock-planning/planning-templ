#include "MissionConstraints.hpp"
#include <gecode/minimodel.hh>
#include "utils/Converter.hpp"
#include <organization_model/Algebra.hpp>

namespace templ {
namespace solvers {
namespace csp {


void MissionConstraints::allDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel)
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ roles.size(), /*height --> row*/ requirements.size());

    for(size_t roleIndex = 0; roleIndex < roles.size(); ++roleIndex)
    {
        const Role& role = roles[roleIndex];
        if(role.getModel() == roleModel);
        {
            Gecode::IntVarArgs args;
            {
                size_t fluent = FluentTimeResource::getIndex(requirements, fts0);
                Gecode::IntVar v = roleDistribution(roleIndex, fluent);
                args << v;
            }
            {
                size_t fluent = FluentTimeResource::getIndex(requirements, fts1);
                Gecode::IntVar v = roleDistribution(roleIndex, fluent);
                args << v;
            }

            Gecode::rel(home, Gecode::sum(args) <= 1);
        }
    }
}

void MissionConstraints::minDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel, uint32_t minDistinctRoles)
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
        //rel(home, v0, Gecode::IRT_EQ, 0); //minDistinctRoles);
        //rel(home, v1, Gecode::IRT_EQ, 0); //minDistinctRoles);

        //LOG_WARN_S << "v0" << v0 << " at index: " << roleIndex << "/" << fluent0;
        //LOG_WARN_S << "v1" << v1 << " at index: " << roleIndex << "/" << fluent1;
     }
     LOG_WARN_S << "Min distinct between: " << fts0.toString(4) << std::endl
         << fts1.toString(4)
         << minDistinctRoles
         << " args: "<< args;
     rel(home, sum(args) >= minDistinctRoles);
     //rel(home, sum(args) == 0); //minDistinctRoles);
     //if( home.failed())
     //{
     //    assert(false);
     //}
}

void MissionConstraints::addDistinct(Gecode::Space& home, Gecode::IntVarArray& roleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel, uint32_t additional)
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ roles.size(), /*height --> row*/ requirements.size());

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

std::set<Role> MissionConstraints::getUniqueRoles(const Gecode::IntVarArray& constRoleUsage,
        const Role::List& roles, const std::vector<FluentTimeResource>& requirements,
        const FluentTimeResource& fts0, const FluentTimeResource& fts1,
        const owlapi::model::IRI& roleModel)
{
    Gecode::IntVarArray& roleUsage = const_cast<Gecode::IntVarArray&>(constRoleUsage);
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(roleUsage, /*width --> col*/ roles.size(), /*height --> row*/ requirements.size());

    std::set<Role> assignedRoles;

    // Check if resource requirements holds
    for(size_t i = 0; i < requirements.size(); ++i)
    {
        for(size_t r = 0; r < roles.size(); ++r)
        {
            const Gecode::IntVar& var = roleDistribution(r, i);
            if(!var.assigned())
            {
                throw std::runtime_error("templ::solvers::csp::MissionConstraints::getUniqueRoles: can only be performed on a fully assigned array. Failed for role: '" + roles[r].toString() + "'");
            }

            Gecode::IntVarValues v( var );
            if( v.val() == 1 )
            {
                if(requirements[i] == fts0 || requirements[i] == fts1)
                {
                    assignedRoles.insert( roles[r] );
                }
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
            // add function requirement --> check addFunctionRequirement
            // update FTR modelpool
            // (add max cardinalities)
            std::cout << "Updated min cardinalities: for " << model.toString() << std::endl << it->toString(4);
            return;
        }
    }
    throw std::invalid_argument("templ::solvers::csp::MissionConstraints::addModelRequirement: given FluentTimeResource is not part of the given list");
}

void MissionConstraints::addFunctionRequirement(const owlapi::model::IRIList& allAvailableResources, std::vector<FluentTimeResource>& resourceRequirements,
        const FluentTimeResource& fts,
        const owlapi::model::IRI& function,
        organization_model::OrganizationModelAsk ask)
{
    // Find the function requirement index
    size_t index = 0;
    owlapi::model::IRIList::const_iterator cit = allAvailableResources.begin();
    for(; cit != allAvailableResources.end(); ++cit, ++index)
    {
        if(*cit == function)
        {
            break;
        }
    }

    // If function cannot be found add the function to the (known) required resources
    if(index >= allAvailableResources.size())
    {
            throw std::invalid_argument("templ::solvers::csp::MissionConstraints: could not find the resource index for: '" + function.toString() + "' -- which is not a service class");
    }
    LOG_DEBUG_S << "Using resource index: " << index;

    // identify the fluent time resource
    size_t idx = FluentTimeResource::getIndex(resourceRequirements, fts);
    FluentTimeResource& ftr = resourceRequirements[idx];

    LOG_DEBUG_S << "Fluent before adding function requirement: " << ftr.toString();

    // insert the function requirement
    ftr.resources.insert(index);
    ftr.maxCardinalities = organization_model::Algebra::max(ftr.maxCardinalities, ask.getFunctionalSaturationBound(function) );
    LOG_DEBUG_S << "Fluent after adding function requirement: " << ftr.toString();
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ

