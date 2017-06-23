#ifndef TEMPL_SOLVERS_COST_HPP
#define TEMPL_SOLVERS_COST_HPP

#include "../symbols/constants/Location.hpp"
#include <organization_model/OrganizationModel.hpp>
#include "../Role.hpp"

namespace templ {
namespace solvers {

class Cost
{
public:
    Cost(const organization_model::OrganizationModel::Ptr& organizationModel);

    /**
     * Compute the travel distance from a given path
     */
    static double getTravelDistance(const symbols::constants::Location::PtrList& path);

    double estimateTravelTime(const symbols::constants::Location::Ptr& from,
            const symbols::constants::Location::Ptr& to,
            const Role::Set& roles);

private:
    organization_model::OrganizationModel::Ptr mpOrganizationModel;
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_COST_HPP
