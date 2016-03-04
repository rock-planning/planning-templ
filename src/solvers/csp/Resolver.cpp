#include "Resolver.hpp"
#include <templ/MissionPlanner.hpp>

namespace templ {
namespace solvers {
namespace csp {

Resolver::Resolver(const Type& type)
    : mType(type)
{}

RoleAddDistinction::RoleAddDistinction(const FluentTimeResource& fts0, 
        const FluentTimeResource& fts1,
        const owlapi::model::IRI& model,
        uint32_t addDelta,
        const RoleDistribution::Solution& solution)
    : Resolver(ROLE_DISTRIBUTION)
    , mFts0(fts0)
    , mFts1(fts1)
    , mModel(model)
    , mAdd(addDelta)
    , mSolution(solution)
{}

void RoleAddDistinction::apply(MissionPlanner* missionPlanner)
{
    missionPlanner->mRoleDistribution->addDistinct(mFts0, mFts1, mModel, mAdd, mSolution);
    delete missionPlanner->mRoleDistributionSearchEngine;
    missionPlanner->mRoleDistributionSearchEngine = new Gecode::BAB<RoleDistribution>(missionPlanner->mRoleDistribution);
}

FunctionalityRequest::FunctionalityRequest(const symbols::constants::Location::Ptr& location,
        const solvers::temporal::Interval& interval,
        const owlapi::model::IRI& model)
    : Resolver(MODEL_DISTRIBUTION)
    , mLocation(location)
    , mInterval(interval)
    , mModel(model)
{}

void FunctionalityRequest::apply(MissionPlanner* missionPlanner)
{
    missionPlanner->constrainMission(mLocation, mInterval, mModel);
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
