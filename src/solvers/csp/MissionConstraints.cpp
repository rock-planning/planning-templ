#include "MissionConstraints.hpp"
#include <gecode/minimodel.hh>

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

} // end namespace csp
} // end namespace solvers
} // end namespace templ

