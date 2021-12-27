#ifndef _TEMPL_SOLVERS_SOLUTIONSIMULATION_HPP_
#define _TEMPL_SOLVERS_SOLUTIONSIMULATION_HPP_

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

using namespace moreorg;
namespace templ
{
    namespace solvers
    {

        typedef std::pair<ResourceInstance, int> IndividualComponentFailureCount;
        typedef std::pair<SpaceTime::Network::tuple_t::Ptr, owlapi::model::IRI> MissedRequirement;

        struct MinMaxAvg
        {
            MinMaxAvg(double min_, double max_, double avg_)
            {
                min = min_;
                max = max_;
                avg = avg_;
            }
            double min;
            double max;
            double avg;
        };
        struct ComponentFailureResult
        {
            ComponentFailureResult(SpaceTime::Network::tuple_t::Ptr &tuple, const reasoning::ModelBound &requiredModel, const ResourceInstance &failedComponent)
                : requirement(requiredModel), component(failedComponent)
            {
                tupleOfFailure = tuple;
            }
            SpaceTime::Network::tuple_t::Ptr tupleOfFailure;
            reasoning::ModelBound requirement;
            ResourceInstance component;

            typedef std::vector<ComponentFailureResult> List;
        };

        struct ResultAnalysis // make this a class?
        {

            std::vector<IndividualComponentFailureCount> individualComponentFailureCountList; // counts failures of specific components during all simulation runs
            std::map<std::string, double> componentImportanceFactors;                         // includes average missed requirements per execution of components
            std::vector<std::pair<double, MinMaxAvg>> failedToEfficacyTripleList; // list of timepoint-ordered (t_0 = [0]) failed-components-to-efficacy relations (pair<avgFailedComponents, minMaxAvg efficacy at this tp>), is this the way?
            std::map<double, int> efficacyCounts; // map of exact efficacies and there occurrence count
            double avgEfficacy; // avg Efficacy
            std::map<int, std::vector<double>> componentFailuresToEfficacy; // relation of count of failed components and occuring efficacies
        };

        struct SimulationRunResult
        {
            // TODO: Do I want / need all of these data or can I derive them from one / multiple others
            ComponentFailureResult::List simulationFailureResult; // list of component failures with some additional information
            std::vector<IndividualComponentFailureCount> importanceFactors; // list of pair <failed components, future missed requirements>
            std::vector<MissedRequirement> missedRequirements; // list of all missed requirements
            double efficacy; // actual efficacy of this run
            std::vector<std::pair<int, double>> failedToEfficacyTripleList; // ordered by timepoints ([0] = tp[0]), pair = <failed components, efficacy>
        };


        class SolutionSimulation
        {
        private:
            double mNumRuns;
            std::vector<utils::ProbabilityType> mMetricsChainToAnalyze; // ordered vector of metrics to apply -> e.g. first check if metric 1 is "successful", if not: check metric 2, ...
            std::mt19937 mRandomEngine;
            std::vector<SimulationRunResult> mRunResults;
            double mEfficacySuccessThreshold;

        public:
            SolutionSimulation(double numRuns, std::vector<utils::ProbabilityType> metricsChainToAnalyze, double efficacySuccessThreshold);
            // TODO what to return?
            bool run(Mission::Ptr &mission, const SpaceTime::Network &solution, const OrganizationModelAsk &ask, std::map<SpaceTime::Network::tuple_t::Ptr, FluentTimeResource::List> &tupleFtrMap, const temporal::TemporalConstraintNetwork::Assignment &timeAssignment, const std::vector<FluentTimeResource> &resourceRequirements, bool findAlternativeSolution = false);
            std::vector<SimulationRunResult> getRunResults() const { return mRunResults; }
            ResultAnalysis analyzeSimulationResults();
            void saveSimulationResults(std::string filepath = "/tmp/sim_result/");
            SpaceTime::Network planAlternativeSolution(Mission::Ptr &mission, temporal::point_algebra::TimePoint::PtrList &modifiedTimepoints, const moreorg::ResourceInstance::List &componentBlacklist);
        };

    } // end namespace solvers

} // end namespace templ

#endif // _TEMPL_SOLVERS_SOLUTIONSIMULATION_