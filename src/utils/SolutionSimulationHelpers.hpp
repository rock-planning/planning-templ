#ifndef TEMPL_UTILS_SOLUTION_SIMULATION_HELPERS_HPP
#define TEMPL_UTILS_SOLUTION_SIMULATION_HELPERS_HPP

#include "../solvers/SolutionAnalysis.hpp"
#include <moreorg/ResourceInstance.hpp>
#include <moreorg/metrics/Probability.hpp>
#include "../solvers/temporal/TemporalConstraintNetwork.hpp"
#include <map>

namespace templ {
namespace utils {

enum ProbabilityType
{
    UNKNOWN = 0,
    /// Probability of Failure
    POF,
    /// FIXME: - not implemented
    RES
};

solvers::SolutionAnalysis::MinMaxModelPools getRequiredResources(
        const solvers::FluentTimeResource& ftr,
        const std::vector<solvers::FluentTimeResource> &resourceRequirements,
        const moreorg::OrganizationModelAsk& om);

moreorg::ModelPool getMinResourceRequirements(
    const solvers::FluentTimeResource& ftr,
    const std::vector<solvers::FluentTimeResource>& resourceRequirements,
    const moreorg::OrganizationModelAsk& om);

moreorg::ResourceInstance::List getAvailableResources(
    const symbols::constants::Location::Ptr& location,
    const solvers::temporal::point_algebra::TimePoint::Ptr& timepoint,
    const SpaceTime::Network& solutionNetwork,
    const moreorg::OrganizationModelAsk& om);

moreorg::ResourceInstance::List getMinAvailableResources(
    const SpaceTime::Network::tuple_t::Ptr& tuple,
    const SpaceTime::Network& solutionNetwork,
    const moreorg::OrganizationModelAsk& om);

moreorg::metrics::Probability::List initializeProbabilityList(
    const ProbabilityType &type,
    std::map<moreorg::reasoning::ModelBound,
    moreorg::ResourceInstance::List>& resourceAssignmentMap,
    moreorg::ResourceInstance::List& available,
    const moreorg::OrganizationModelAsk& ask,
    double t0,
    double t1);

moreorg::metrics::Probability::List matchResourcesToProbability(
    const ProbabilityType& type,
    const moreorg::OrganizationModelAsk &ask,
    const std::vector<owlapi::model::OWLCardinalityRestriction::Ptr>& required,
    moreorg::ResourceInstance::List& availableAgents,
    double t0,
    double t1);

solvers::temporal::TemporalConstraintNetwork::Assignment quantifyTime(
    templ::Mission::Ptr& mission,
    SpaceTime::Network& solutionNetwork,
    std::vector<solvers::FluentTimeResource>&resourceRequirements);

moreorg::ResourceInstance::List filterResourcesByBlacklist(
    moreorg::ResourceInstance::List& resources,
    moreorg::ResourceInstance::List& blacklist);

moreorg::ModelPool checkAndRemoveRequirements(
    moreorg::ModelPool& available,
    moreorg::ModelPool& required,
    const moreorg::OrganizationModelAsk& ask);

moreorg::ResourceInstance::List filterSingleResource(
    moreorg::ResourceInstance::List& resources,
    moreorg::ResourceInstance& resourceToRemove);

int checkFutureImpactOfAssignment(
    SpaceTime::Network& solution,
    templ::Mission::Ptr& mission,
    moreorg::ResourceInstance& assignment,
    moreorg::ResourceInstance::List& previouslyFailed,
    SpaceTime::Network::tuple_t::Ptr& vertexTuple,
    std::map<SpaceTime::Network::tuple_t::Ptr, solvers::FluentTimeResource::List>& tupleFtrMap,
    moreorg::OrganizationModelAsk& omAsk,
    const std::vector<solvers::FluentTimeResource>& resourceRequirements);

} // end namespace ultils
} // end namespace templ

#endif // TEMPL_UTILS_SOLUTION_SIMULATION_HELPERS_HPP
