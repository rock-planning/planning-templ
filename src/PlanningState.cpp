#include "PlanningState.hpp"
#include <base/Logging.hpp>

namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {

PlanningState::PlanningState(const Mission& mission, const std::string& label)
    : graph_analysis::Vertex(label)
    , mMission(mission)
    , mpModelDistribution(0)
    , mpModelDistributionSearchEngine(0)
    , mpRoleDistribution(0)
    , mpRoleDistributionSearchEngine(0)
{
    mLocations = mMission.getLocations();
    mTimePoints = mMission.getTimepoints();
}

PlanningState::PlanningState(const PlanningState& other)
    : mMission("")
{
    mMission = other.mMission;
    mpTemporalConstraintNetwork = other.mpTemporalConstraintNetwork;
    mpTemporalConstraintNetwork->setGraph(other.mpTemporalConstraintNetwork->getGraph()->cloneEdges());

    mTimePoints = other.mTimePoints;
    mLocations = other.mLocations;

    mpModelDistribution = other.mpModelDistribution;
    mpModelDistributionSearchEngine = other.mpModelDistributionSearchEngine;
    mModelDistributionSolution = other.mModelDistributionSolution;

    mpRoleDistribution = other.mpRoleDistribution;
    mpRoleDistributionSearchEngine = other.mpRoleDistributionSearchEngine;
    mRoleDistributionSolution = other.mRoleDistributionSolution;

    mResolvers = other.mResolvers;
}

PlanningState::~PlanningState()
{
    mpModelDistribution = NULL;
    mpRoleDistribution = NULL;
    //delete mpModelDistribution;
    //delete mpModelDistributionSearchEngine;

    //delete mpRoleDistribution;
    //delete mpRoleDistributionSearchEngine;
}

void PlanningState::setMission(const Mission& mission)
{
    mMission = mission;
    mLocations = mission.getLocations();
    mTimePoints = mission.getTimepoints();
}
void PlanningState::setTemporalConstraintNetwork(const solvers::temporal::TemporalConstraintNetwork::Ptr& tcn)
{ 
    mpTemporalConstraintNetwork = tcn;
    mpTemporalConstraintNetwork->setGraph(mpTemporalConstraintNetwork->getGraph()->cloneEdges());
    mpTemporalConstraintNetwork->sort(mTimePoints);
    LOG_INFO_S << pa::TimePoint::toString(mTimePoints);

}

void PlanningState::cleanup(SubStateFlag flag)
{
    if(flag & TEMPORAL)
    {
        // nothing to cleanup
        flag = MODEL;
    } 
   
    if(flag & MODEL)
    {
        mpModelDistribution = NULL;
        mpModelDistributionSearchEngine = NULL;
        flag = ROLE;
    }

    if(flag & ROLE)
    {
        mpRoleDistribution = NULL;
        mpRoleDistributionSearchEngine = NULL;
    }
}

void PlanningState::constrainMission(const symbols::constants::Location::Ptr& location,
        const solvers::temporal::Interval& interval,
        const owlapi::model::IRI& model,
        uint32_t cardinality,
        owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType cardinalityRestrictionType)
{
    LOG_WARN_S << "CONSTRAINING MISSION " 
        << mMission.getName() 
        << " at:" 
        << location->toString() 
        << "[" << interval.toString() << "]"
        << " model: " << model.toString() 
        << " cardinality: " << cardinality
        << " (" << owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionTypeTxt[cardinalityRestrictionType] << ")";

    mMission.addResourceLocationCardinalityConstraint(location,
            interval.getFrom(), interval.getTo(),
            model, cardinality, cardinalityRestrictionType);
}

void PlanningState::removeResolver(const solvers::csp::Resolver::Ptr& resolver)
{
    using namespace solvers::csp;
    std::vector<Resolver::Ptr>::iterator rit = std::find_if(mResolvers.begin(), mResolvers.end(), [resolver](const Resolver::Ptr& r)
            {
                return r == resolver;
            });

    if(rit != mResolvers.end())
    {
        mResolvers.erase(rit);
    } else {
        throw std::runtime_error("templ::PlanningState::removeResolver: could not find resolver in planning state");
    }
}

} // end namespace templ
