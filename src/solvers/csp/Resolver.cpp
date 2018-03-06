#include "Resolver.hpp"
#include <templ/PlanningState.hpp>

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

void RoleAddDistinction::apply(PlanningState* planningState)
{
    // Provide a deep copy
    RoleDistribution* rd = new RoleDistribution(*planningState->getRoleDistribution());
    rd->addDistinct(mFts0, mFts1, mModel, mAdd, mSolution);
    planningState->setRoleDistribution(rd);
    planningState->setRoleDistributionSearchEngine(new Gecode::BAB<RoleDistribution>(rd));
}

FunctionalityRequest::FunctionalityRequest(const symbols::constants::Location::Ptr& location,
        const solvers::temporal::Interval& interval,
        const owlapi::model::IRI& model)
    : Resolver(MODEL_DISTRIBUTION)
    , mLocation(location)
    , mInterval(interval)
    , mModel(model)
{}

void FunctionalityRequest::apply(PlanningState* state)
{
    state->constrainMission(mLocation, mInterval, mModel);
    // enforce triggering a new model distribution due to new constraints
    state->cleanup(PlanningState::MODEL);
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
