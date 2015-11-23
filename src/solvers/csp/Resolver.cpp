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
    //missionPlanner->mRoleDistribution->addDistinct(mFts0, mFts1, mModel, mAdd, mSolution);
    missionPlanner->mRoleDistribution->allDistinct(mFts0, mFts1, mModel);
    delete missionPlanner->mRoleDistributionSearchEngine;
    missionPlanner->mRoleDistributionSearchEngine = new Gecode::BAB<RoleDistribution>(missionPlanner->mRoleDistribution);
}

FunctionalityRequest::FunctionalityRequest(const FluentTimeResource& fts,
        const owlapi::model::IRI& model)
    : Resolver(MODEL_DISTRIBUTION)
    , mFts(fts)
    , mModel(model)
{}

void FunctionalityRequest::apply(MissionPlanner* missionPlanner)
{
    missionPlanner->mModelDistribution->addFunctionRequirement(mFts, mModel);
    delete missionPlanner->mModelDistributionSearchEngine;
    missionPlanner->mModelDistributionSearchEngine = new Gecode::BAB<ModelDistribution>(missionPlanner->mModelDistribution);
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
