#include "Cost.hpp"
#include <moreorg/facades/Robot.hpp>
#include <moreorg/OrganizationModelAsk.hpp>
#include <moreorg/Agent.hpp>

using namespace moreorg;

namespace templ {
namespace solvers {

Cost::Cost(const moreorg::OrganizationModelAsk& organizationModelAsk,
        const owlapi::model::IRI& mobilityFunctionality,
        double feasibilityCheckTimeoutInMs)
    : mOrganizationModelAsk(organizationModelAsk)
    , mFeasibilityCheckTimeoutInMs(feasibilityCheckTimeoutInMs)
{
    mMoveToResource.insert( moreorg::Resource( mobilityFunctionality ) );
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
    if(distance < 1E-06)
    {
        return 0;
    }

    ModelPool modelPool = Role::getModelPool(coalition);

    // Identify systems that should be combined for the transport
    ModelPool::List coalitionStructure =
        mOrganizationModelAsk.findFeasibleCoalitionStructure(modelPool, mMoveToResource, mFeasibilityCheckTimeoutInMs);

    if(coalitionStructure.empty())
    {
        LOG_WARN_S << "Infeasible transition for model pool "
            << modelPool.toString(4)
            << " from " << from->toString()
            << " to " << to->toString()
            << " timeout: " << mFeasibilityCheckTimeoutInMs;

        return std::numeric_limits<double>::max();
    }

    double minTime = 0;
    for(const ModelPool& coalition : coalitionStructure)
    {
        facades::Robot robot = moreorg::facades::Robot::getInstance(coalition, mOrganizationModelAsk);
        if( robot.isMobile())
        {
            double time = distance / robot.getNominalVelocity();
            minTime = std::max(time, minTime);
        }
    }
    return minTime;
}

} // end namespace solvers
} // end namespace templ
