#ifndef TEMPL_SOLVERS_SOLUTION_SIMULATION_HPP
#define TEMPL_SOLVERS_SOLUTION_SIMULATION_HPP

#include "FluentTimeResource.hpp"
#include "SolutionAnalysis.hpp"
#include "../Mission.hpp"
#include "../SpaceTime.hpp"
#include "../utils/SolutionSimulationHelpers.hpp"
#include <moreorg/Metric.hpp>
#include <moreorg/metrics/Redundancy.hpp>
#include <moreorg/ResourceInstance.hpp>
#include <moreorg/vocabularies/OM.hpp>
#include <owlapi/model/OWLCardinalityRestriction.hpp>
#include <vector>
#include <random>

namespace templ {
namespace solvers {

using IndividualComponentFailureCount = std::pair<moreorg::ResourceInstance, int>;
using MissedRequirement = std::pair<SpaceTime::Network::tuple_t::Ptr, owlapi::model::IRI>;

struct MinMaxAvg
{
    MinMaxAvg(double min, double max, double avg);

    double min;
    double max;
    double avg;
};

struct ComponentFailureResult
{
    ComponentFailureResult(SpaceTime::Network::tuple_t::Ptr& tuple,
                           const moreorg::reasoning::ModelBound& requiredModel,
                           const moreorg::ResourceInstance& failedComponent);

    SpaceTime::Network::tuple_t::Ptr tupleOfFailure;
    moreorg::reasoning::ModelBound requirement;
    moreorg::ResourceInstance component;

    typedef std::vector<ComponentFailureResult> List;
};

struct ResultAnalysis // make this a class?
{
    // counts failures of specific components during all simulation runs
    std::vector<IndividualComponentFailureCount> individualComponentFailureCountList;

    // includes average missed requirements per execution of components
    std::map<std::string, double> componentImportanceFactors;

    // list of timepoint-ordered (t_0 = [0]) failed-components-to-efficacy relations (pair<avgFailedComponents, minMaxAvg efficacy at this tp>), is this the way?
    std::vector<std::pair<double, MinMaxAvg>> failedToEfficacyTripleList;

    // map of exact efficacies and there occurrence count
    std::map<double, int> efficacyCounts;

    // avg Efficacy
    double avgEfficacy;

    // relation of count of failed components and occuring efficacies
    std::map<int, std::vector<double>> componentFailuresToEfficacy;
};

struct SimulationRunResult
{
    // TODO: Do I want / need all of these data or can I derive them from one / multiple others
    // list of component failures with some additional information
    ComponentFailureResult::List simulationFailureResult;

    // list of pair <failed components, future missed requirements>
    std::vector<IndividualComponentFailureCount> importanceFactors;

    // list of all missed requirements
    std::vector<MissedRequirement> missedRequirements;

    // actual efficacy of this run
    double efficacy;

    // ordered by timepoints ([0] = tp[0]), pair = <failed components, efficacy>
    std::vector<std::pair<int, double>> failedToEfficacyTripleList;
};


class SolutionSimulation
{
private:
    double mNumRuns;

    // ordered vector of metrics to apply -> e.g. first check if metric 1 is "successful", if not: check metric 2, ...
    std::vector<utils::ProbabilityType> mMetricsChainToAnalyze;
    std::mt19937 mRandomEngine;
    std::vector<SimulationRunResult> mRunResults;
    double mEfficacySuccessThreshold;

public:
    /**
     * \param numRuns Number of simulation run
     * \param metricsChainToAnalyze List of probability types to use for the analysis
     */
    SolutionSimulation(double numRuns,
                       std::vector<utils::ProbabilityType> metricsChainToAnalyze,
                       double efficacySuccessThreshold);

    // TODO what to return?
    bool run(Mission::Ptr& mission,
             const SpaceTime::Network& solution,
             const moreorg::OrganizationModelAsk& ask,
             std::map<SpaceTime::Network::tuple_t::Ptr,
             FluentTimeResource::List>& tupleFtrMap,
             const temporal::TemporalConstraintNetwork::Assignment& timeAssignment,
             const std::vector<FluentTimeResource>& resourceRequirements,
             bool findAlternativeSolution = false);

    std::vector<SimulationRunResult> getRunResults() const { return mRunResults; }

    ResultAnalysis analyzeSimulationResults();

    void saveSimulationResults(std::string filepath = "/tmp/sim_result/");

    SpaceTime::Network planAlternativeSolution(Mission::Ptr& mission,
                 temporal::point_algebra::TimePoint::PtrList& modifiedTimepoints,
                 const moreorg::ResourceInstance::List& componentBlacklist);
};

} // end namespace solvers
} // end namespace templ

#endif // TEMPL_SOLVERS_SOLUTION_SIMULATION
