#include "SolutionSimulation.hpp"

//#include "utils/SolutionModifier.hpp"
#include "Solution.hpp"
#include <moreorg/Algebra.hpp>
#include "csp/TransportNetwork.hpp"
#include "../utils/CSVLogger.hpp"
#include <fstream>

using namespace moreorg;
using namespace graph_analysis;
using namespace templ::solvers::temporal::point_algebra;
using namespace templ::symbols::constants;

namespace templ {
namespace solvers {

MinMaxAvg::MinMaxAvg(double min, double max, double avg)
    : min(min)
    , max(max)
    , avg(avg)
{}

ComponentFailureResult::ComponentFailureResult(SpaceTime::Network::tuple_t::Ptr& tuple,
                           const reasoning::ModelBound& requiredModel,
                           const ResourceInstance& failedComponent)
    : tupleOfFailure(tuple)
    , requirement(requiredModel)
    , component(failedComponent)
{}



SolutionSimulation::SolutionSimulation(double numRuns,
     std::vector<utils::ProbabilityType> metricsChainToAnalyze,
     double efficacySuccessThreshold)
        : mNumRuns(numRuns)
        , mMetricsChainToAnalyze(metricsChainToAnalyze)
        , mEfficacySuccessThreshold(efficacySuccessThreshold)
{}

bool SolutionSimulation::run(const Mission::Ptr& mission,
     const SpaceTime::Network& solution,
     bool findAlternativeSolution)
{
    std::map<SpaceTime::Network::tuple_t::Ptr, FluentTimeResource::List> tupleFtrMap;
    FluentTimeResource::List resourceRequirements = Mission::getResourceRequirements(mission);
    for (const solvers::FluentTimeResource &ftr : resourceRequirements)
    {
        SpaceTime::Network::tuple_t::PtrList tuples = solution.getTuples(ftr.getInterval().getFrom(),
                                                                         ftr.getInterval().getTo(),
                                                                         ftr.getLocation());

        for (const SpaceTime::Network::tuple_t::Ptr &tuple : tuples)
        {
            tupleFtrMap[tuple].push_back(ftr);
        }
    }

    solvers::temporal::TemporalConstraintNetwork::Assignment timeAssignment = utils::quantifyTime(mission, solution, resourceRequirements);

    // init mission and solution relevant stuff
    TimePoint::PtrList timepoints = mission->getTimepoints();
    Location::PtrList locations = mission->getLocations();
    OrganizationModelAsk omAsk = mission->getOrganizationModelAsk();
    SpaceTime::Network currentSolution(solution);

    if(mMetricsChainToAnalyze.empty())
    {
        throw std::runtime_error("SolutionSimulation:: no metrics to analyze");
    }

    //init some used variables for results
    int is_start = 0;
    ResourceInstance::List failedComponentsList;
    int requirement_count = 0;
    int requirements_success = 0;

    SimulationRunResult runResult;
    TimePoint::PtrList modifiedTimepoints(mission->getTimepoints());

    // Start Simulation
    // Iterate over all space-timepoints
    for (const TimePoint::Ptr& tp : timepoints)
    {
        if (is_start < 2)
        {
            // How to handle this properly?
            runResult.failedToEfficacyTripleList.push_back(std::make_pair(0, 1.));
            ++is_start;
            continue;
        }

        for (const Location::Ptr& location : locations)
        {
            // Get Space-TimePoint (stp)
            SpaceTime::Network::tuple_t::Ptr vertexTuple = currentSolution.tupleByKeys(location, tp);
            std::vector<Edge::Ptr> inEdges = currentSolution.getGraph()->getInEdges(vertexTuple);

            // If there are no inedges, this stp is irrelevant, since it is not part of the solution
            if (inEdges.empty())
            {
                continue;
            }

            // TODO: For now, one can assume that all edges start at the same (previous) timepoint
            SpaceTime::Network::tuple_t::Ptr fromTuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(inEdges.front()->getSourceVertex());

            // Retrieve exact timeslot that the edge covers
            double start_time = timeAssignment.find(fromTuple->second())->second;
            double end_time = timeAssignment.find(vertexTuple->second())->second;

            std::map<reasoning::ModelBound, ResourceInstance::List> failedComponentsMap;
            ModelPool required;
            // prepare required resources
            for (const FluentTimeResource& ftr : tupleFtrMap[vertexTuple])
            {
                if (required.empty())
                {
                    required = utils::getMinResourceRequirements(ftr, resourceRequirements, omAsk);
                } else
                {
                    ModelPool minRequired = utils::getMinResourceRequirements(ftr, resourceRequirements, omAsk);
                    required = Algebra::merge(minRequired, required);
                }
            }

            // no requirements? Skip this tuple ...
            if (required.empty())
            {
                continue;
            }

            // count requirements for efficacy calculation
            requirement_count += (int) required.size();

            ResourceInstance::List availableUnfiltered = utils::getMinAvailableResources(vertexTuple, currentSolution, omAsk);
            // Remove the already failed components from the list of available
            ResourceInstance::List available = utils::filterResourcesByBlacklist(availableUnfiltered, failedComponentsList);

            // Compute the Metric(s)
            using namespace metrics;
            std::vector<Probability::List> metricList;
            ModelPool availableModelPool = ResourceInstance::toModelPool(available);
            ModelPool finalRequired = utils::checkAndRemoveRequirements(availableModelPool, required, omAsk);
            ModelPoolDelta delta = Algebra::delta(finalRequired, required);
            for (auto &r : required)
            {
                if ((delta[r.first]) > 0)
                {
                    // std::cout << "Missing requirement: " << r.first.toString() << " in tuple: " << vertexTuple->toString() << std::endl << "available: " << availableModelPool.toString() << std::endl;
                    runResult.missedRequirements.push_back(std::make_pair(vertexTuple, r.first));
                }
            }

            if (finalRequired.empty())
            {
                continue;
            }

            std::vector<owlapi::model::OWLCardinalityRestriction::Ptr> r_required = omAsk.getRequiredCardinalities(finalRequired, vocabulary::OM::has());
            for (const auto &mt : mMetricsChainToAnalyze)
            {
                try
                {
                    // try to match available and required resources and init probabilities
                    metricList.push_back(utils::matchResourcesToProbability(mt, omAsk, r_required, available, start_time, end_time));
                }
                catch (const std::exception &e)
                {
                    // Requirements couldnt be fullfilled
                    // TODO handle properly
                    std::cout << "failed: " << e.what() << std::endl;
                    //std::cout << "Resource requirements were: " << finalRequired.toString() << " but only: " << availableModelPool.toString() << " were available." << std::endl;
                }
            }

            if (metricList.empty())
            {
                continue;
            }

            for (auto& probability : metricList[0])
            {
                // assume that given Probability Function gives probability for something to fail
                double p = probability.getProbabilityDensityFunction()->getConditional(start_time, end_time);
                std::bernoulli_distribution d(p);
                int cardinality = probability.getRequirement().min;
                int sampledAndSurvived = 0;
                for (auto &assignment : probability.getAssignments())
                {
                    if (sampledAndSurvived >= cardinality)
                        break;
                    if (d(mRandomEngine))
                    {
                        std::cout << "Component: " << assignment.toString() << " failed with p: " << p << std::endl;
                        failedComponentsMap[probability.getRequirement()].push_back(assignment);
                        failedComponentsList.push_back(assignment); // rework needed
                        int count = utils::checkFutureImpactOfAssignment(currentSolution, mission, assignment, failedComponentsList, vertexTuple, tupleFtrMap, omAsk, resourceRequirements);
                        runResult.importanceFactors.push_back(std::make_pair(assignment, count));
                        if ((count > 0) && findAlternativeSolution)
                        {
                            std::cout << count << std::endl;
                            // plan and analyse alternative solution
                            try
                            {
                                planAlternativeSolution(mission, modifiedTimepoints, failedComponentsList);
                            }
                            catch (const std::exception &e)
                            {
                                std::cout << "Exception during alternative solution planning: " << e.what() << '\n';
                            }
                        }
                    }
                    else
                    {
                        sampledAndSurvived++;
                    }
                }
            }

            if (metricList.size() > 1)
            {
                // check other metrics for survival
            }
            ModelPool availableModelPoolAfterSurvival = ResourceInstance::toModelPool(available);
            ModelPoolDelta deltaAfterSurvival = Algebra::delta(availableModelPoolAfterSurvival, required);
            for (auto &r : required)
            {
                if ((delta[r.first]) >= 0)
                {
                    ++requirements_success;
                }
                else
                {
                    runResult.missedRequirements.push_back(std::make_pair(vertexTuple, r.first));
                }
            }
            for (const auto &failedPair : failedComponentsMap)
            {
                //failedComponentsList.insert(failedComponentsList.end(), failedPair.second.begin(), failedPair.second.end());
                for (auto &comp : failedPair.second)
                {
                    ComponentFailureResult cfp(vertexTuple, failedPair.first, comp);
                    runResult.simulationFailureResult.push_back(cfp);
                }
            }
            // modify available resources in solution
        }


        double tempEfficacy = requirements_success/(1.*requirement_count);
        runResult.failedToEfficacyTripleList.push_back(std::make_pair((int)failedComponentsList.size(), tempEfficacy));
        modifiedTimepoints.erase(modifiedTimepoints.begin());
    }

    for (auto &a : failedComponentsList)
    {
        std::cout << "Failed Components: ";
        std::cout << a.toString() << std::endl;
    }

    double efficacy = (double)requirements_success / (double)requirement_count;
    std::cout << "Requirement count: " << requirement_count << " and " << requirements_success << " succesful." << std::endl;
    std::cout << "Efficacy: " << efficacy << std::endl;

    runResult.efficacy = efficacy;
    mRunResults.push_back(runResult);


    if (efficacy < mEfficacySuccessThreshold)
    {
        std::cout << "Failure" << std::endl;
        return false;
    }
    else
    {
        std::cout << "Successful" << std::endl;
        return true;
    }
}

ResultAnalysis SolutionSimulation::analyzeSimulationResults()
{
    ResultAnalysis resultAnalysis;
    int run = 0;
    // iterate through all results
    for(SimulationRunResult&result : mRunResults)
    {
        // iterate through all failed components of the run
        for (auto &componentFailure : result.simulationFailureResult)
        {
            auto it = std::find_if(resultAnalysis.individualComponentFailureCountList.begin(), resultAnalysis.individualComponentFailureCountList.end(), [&componentFailure](IndividualComponentFailureCount icfc)
                                   { return ((componentFailure.component.getName() == icfc.first.getName()) && (componentFailure.component.getAtomicAgent() == icfc.first.getAtomicAgent())); });
            if (it == resultAnalysis.individualComponentFailureCountList.end())
            {
                IndividualComponentFailureCount pair(componentFailure.component, 1);
                resultAnalysis.individualComponentFailureCountList.push_back(pair);
            }
            else
            {
                it->second += 1;
            }
        }

        for (size_t i = 0; i < result.failedToEfficacyTripleList.size(); i++)
        {
            if (run < 1)
            {
                MinMaxAvg minMaxAvg(result.failedToEfficacyTripleList[i].second, result.failedToEfficacyTripleList[i].second, result.failedToEfficacyTripleList[i].second);
                resultAnalysis.failedToEfficacyTripleList.push_back(std::make_pair(result.failedToEfficacyTripleList[i].first, minMaxAvg));
            }
            else
            {
                resultAnalysis.failedToEfficacyTripleList[i].first += result.failedToEfficacyTripleList[i].first;
                resultAnalysis.failedToEfficacyTripleList[i].second.avg += result.failedToEfficacyTripleList[i].second;
                resultAnalysis.failedToEfficacyTripleList[i].second.avg = resultAnalysis.failedToEfficacyTripleList[i].second.avg / 2;
                resultAnalysis.failedToEfficacyTripleList[i].second.min = std::min(resultAnalysis.failedToEfficacyTripleList[i].second.min, result.failedToEfficacyTripleList[i].second);
                resultAnalysis.failedToEfficacyTripleList[i].second.max = std::max(resultAnalysis.failedToEfficacyTripleList[i].second.max, result.failedToEfficacyTripleList[i].second);
            }
        }

        if (resultAnalysis.efficacyCounts[result.efficacy])
        {
            resultAnalysis.efficacyCounts[result.efficacy] += 1;
        }
        else
        {
            resultAnalysis.efficacyCounts[result.efficacy] = 1;
        }
        // Component Failures To Efficacy Metric:
        int index = result.simulationFailureResult.size();
        if (resultAnalysis.componentFailuresToEfficacy.find(index) != resultAnalysis.componentFailuresToEfficacy.end())
        {
            resultAnalysis.componentFailuresToEfficacy[index][0] += 1;
            resultAnalysis.componentFailuresToEfficacy[index][1] += result.efficacy;
            resultAnalysis.componentFailuresToEfficacy[index][1] = resultAnalysis.componentFailuresToEfficacy[index][1] / 2;
            resultAnalysis.componentFailuresToEfficacy[index][2] = std::min(resultAnalysis.componentFailuresToEfficacy[index][2], result.efficacy);
            resultAnalysis.componentFailuresToEfficacy[index][3] = std::max(resultAnalysis.componentFailuresToEfficacy[index][3], result.efficacy);
        }
        else
        {
            std::vector<double> vec;
            vec.push_back(1.);
            vec.push_back(result.efficacy);
            vec.push_back(result.efficacy);
            vec.push_back(result.efficacy);
            resultAnalysis.componentFailuresToEfficacy.insert(std::make_pair(index, vec));
        }

        for (auto &c : result.importanceFactors)
        {
            std::string keyName(c.first.getAtomicAgent().getName() + ":" + c.first.getName().toString().erase(0, 37));
            if (resultAnalysis.componentImportanceFactors[keyName])
            {
                resultAnalysis.componentImportanceFactors[keyName] += c.second;
                resultAnalysis.componentImportanceFactors[keyName] = resultAnalysis.componentImportanceFactors[keyName] / 2;
            }
            else
            {
                resultAnalysis.componentImportanceFactors[keyName] = c.second;
            }
        }

        resultAnalysis.avgEfficacy += result.efficacy;

        ++run;
    }

    for (size_t i = 0; i < resultAnalysis.failedToEfficacyTripleList.size(); i++)
    {
        resultAnalysis.failedToEfficacyTripleList[i].first = resultAnalysis.failedToEfficacyTripleList[i].first / mRunResults.size(); // does this even make sense
    }

    // for (size_t i = 0; i < resultAnalysis.componentFailuresToEfficacy.size(); i++)
    // {
    //     resultAnalysis.componentFailuresToEfficacy[i].second = resultAnalysis.componentFailuresToEfficacy[i].second / resultAnalysis.componentFailuresToEfficacy[i].first;
    // }

    resultAnalysis.avgEfficacy = resultAnalysis.avgEfficacy / mRunResults.size();

    return resultAnalysis;
}

SpaceTime::Network SolutionSimulation::planAlternativeSolution(const Mission::Ptr& mission,
        temporal::point_algebra::TimePoint::PtrList& modifiedTimepoints,
        const moreorg::ResourceInstance::List &componentBlacklist)
{
    using namespace temporal::point_algebra;
    using namespace symbols::constants;
    Mission newMission(mission->getOrganizationModel());
    TimePoint::PtrList timepoints = mission->getTimepoints();
    std::vector<Constraint::Ptr> constraints = mission->getConstraints();

    for (auto &c : constraints)
    {
        QualitativeTimePointConstraint::Ptr tc = dynamic_pointer_cast<QualitativeTimePointConstraint>(c);
        auto it = std::find_if(modifiedTimepoints.begin(), modifiedTimepoints.end(), [&tc](TimePoint::Ptr tp)
                               { return (tc->getSourceVariable()->getLabel() == tp->getLabel()); });
        if (it != modifiedTimepoints.end())
        {
            newMission.addConstraint(tc);
        }
    }
    std::vector<FluentTimeResource> resources = Mission::getResourceRequirements(mission);
    for (auto &ftr : resources)
    {
        TimePoint::Ptr fromTp = ftr.getInterval().getFrom();
        TimePoint::Ptr toTp = ftr.getInterval().getTo();
        auto toIt = std::find_if(modifiedTimepoints.begin(), modifiedTimepoints.end(), [&toTp](TimePoint::Ptr tp)
                                 { return (toTp->getLabel() == tp->getLabel()); });
        if (toIt == modifiedTimepoints.end())
        {
            continue; // skip resource requirement, if outside of used timepoints
        }

        auto it = std::find_if(modifiedTimepoints.begin(), modifiedTimepoints.end(), [&fromTp](TimePoint::Ptr tp)
                               { return (fromTp->getLabel() == tp->getLabel()); });
        if (it == modifiedTimepoints.end())
        {
            fromTp = modifiedTimepoints[0];
        }

        for (auto &m : ftr.getMinCardinalities())
        {
            newMission.addResourceLocationCardinalityConstraint(ftr.getLocation(), fromTp, toTp, m.first, m.second);
        }
    }
    newMission.prepareTimeIntervals();
    Mission::Ptr pNewMission = make_shared<Mission>(newMission);
    moreorg::ModelPool modelPool = mission->getAvailableResources();
    pNewMission->setAvailableResources(modelPool);
    qxcfg::Configuration config;
    config.setValue("TransportNetwork/search/options/total_timeout_in_s", "60");
    std::vector<csp::TransportNetwork::Solution> solutions = csp::TransportNetwork::solve(pNewMission, 1, config, componentBlacklist);

    if (solutions.empty())
    {
        std::cout << "No alternative solutions were found" << std::endl;
    }

    return SpaceTime::Network();
}

void SolutionSimulation::saveSimulationResults(std::string filepath)
{
    ResultAnalysis resultAnalysis = analyzeSimulationResults();
    CSVLogger csvLoggerEfficacyTriple({"timepoint", "failedComponents", "avgEfficacy", "minEfficacy", "maxEfficacy"});

    for (size_t i = 0; i < resultAnalysis.failedToEfficacyTripleList.size(); ++i)
    {
        csvLoggerEfficacyTriple.addToRow(i, "timepoint");
        csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].first, "failedComponents");
        csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.avg, "avgEfficacy");
        csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.min, "minEfficacy");
        csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.max, "maxEfficacy");
        csvLoggerEfficacyTriple.commitRow();
    }
    csvLoggerEfficacyTriple.save(filepath + "failedToEfficacyTripleResult.log");

    CSVLogger csvEfficacies({"efficacy", "count"});
    for (auto &e : resultAnalysis.efficacyCounts)
    {
        csvEfficacies.addToRow(e.first, "efficacy");
        csvEfficacies.addToRow(e.second, "count");
        csvEfficacies.commitRow();
    }
    csvEfficacies.save(filepath + "efficacyCountsResult.log");
    std::string filename(filepath + "componentFailures.log");
    std::ofstream outfile(filename, std::ofstream::out);
    outfile << "component count " << std::endl;
    for (auto &a : resultAnalysis.individualComponentFailureCountList)
    {
        std::cout << "Component: " << a.first.getName().toString() << " failed " << a.second << " times out of " << mNumRuns << " runs." << std::endl;
        outfile << a.first.getAtomicAgent().getName() << ":" << a.first.getName().toString().erase(0, 37) << " " << a.second << std::endl;
    }
    outfile.close();

    CSVLogger csvComponentFailuresToEfficacy({"count", "avgEfficacy", "minEfficacy", "maxEfficacy"});
    for (auto &c : resultAnalysis.componentFailuresToEfficacy)
    {
        csvComponentFailuresToEfficacy.addToRow(c.first, "count");
        csvComponentFailuresToEfficacy.addToRow(c.second[1], "avgEfficacy");
        csvComponentFailuresToEfficacy.addToRow(c.second[2], "minEfficacy");
        csvComponentFailuresToEfficacy.addToRow(c.second[3], "maxEfficacy");
        csvComponentFailuresToEfficacy.commitRow();
    }
    csvComponentFailuresToEfficacy.save(filepath + "componentFailuresToEfficacy.log");

    std::ofstream importanceFactorOut(filepath + "importanceFactors.log", std::ofstream::out);
    importanceFactorOut << "component importanceFactor " << std::endl;
    for (auto &l : resultAnalysis.componentImportanceFactors)
    {
        importanceFactorOut << l.first << " " << l.second << std::endl;
    }
    importanceFactorOut.close();
    std::ofstream statsOut(filepath + "stats.log", std::ofstream::out);
    statsOut << "Efficacy: " << resultAnalysis.avgEfficacy;
    statsOut.close();
}

} // end namespace solvers
} // end namespace templ
