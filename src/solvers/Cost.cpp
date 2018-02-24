#include "Cost.hpp"
#include <organization_model/facades/Robot.hpp>
#include <organization_model/OrganizationModelAsk.hpp>

namespace templ {
namespace solvers {

Cost::Cost(const organization_model::OrganizationModel::Ptr& organizationModel)
    : mpOrganizationModel(organizationModel)
{
}

double Cost::getTravelDistance(const symbols::constants::Location::PtrList& path)
{
    if(path.size() < 2)
    {
        LOG_INFO_S << "templ::solvers::Cost::getTravelDistance:"
                << " not enough locations available to compute travel distance";
        return 0;
    }

    using namespace ::templ::symbols;
    constants::Location::Ptr fromLocation;
    constants::Location::Ptr toLocation;

    double totalDistance = 0.0;
    for(size_t i = 0; i < path.size() -1; ++i)
    {
        fromLocation = path[i];
        toLocation = path[i+1];

        totalDistance += (toLocation->getPosition() - fromLocation->getPosition()).norm();
    }

    return totalDistance;
}


double Cost::estimateTravelTime(const symbols::constants::Location::Ptr& from,
        const symbols::constants::Location::Ptr& to,
        const Coalition& coalition)
{
    assert(!coalition.empty());

    double distance = (from->getPosition() - to->getPosition()).norm();

    organization_model::ModelPool modelPool = Role::getModelPool(coalition);
    organization_model::OrganizationModelAsk ask(mpOrganizationModel, modelPool, true);

    // Identify system that should be used for the transport
    double minTime = std::numeric_limits<double>::max();
    organization_model::facades::Robot robot(modelPool, ask);
    if( robot.isMobile())
    {
        double time = distance / robot.getNominalVelocity();
        minTime = std::min(time, minTime);
        LOG_WARN_S << "RoleSet: " << std::endl << Role::toString(coalition) << std::endl <<
            " time " << minTime;
    } else {
        LOG_WARN_S << "Immobile RoleSet: " << std::endl << Role::toString(coalition) << std::endl <<
            " time " << minTime;
    }
    return minTime;
}

} // end namespace solvers
} // end namespace templ
