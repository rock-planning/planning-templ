#include <boost/test/unit_test.hpp>
#include <templ/solvers/SolutionSimulation.hpp>
#include <map>
#include <templ/io/MissionReader.hpp>
#include <moreorg/OrganizationModel.hpp>
#include "../test_utils.hpp"
#include <string>
#include <templ/utils/CSVLogger.hpp>
#include <fstream>

using namespace moreorg;
using namespace templ;


struct SimulationFixture1
{
    SimulationFixture1()
    {
        organizationModel = OrganizationModel::getInstance(getRootDir() + "/test/data/solution_simulation/organization_model/om-project-transterra.owl");
        // todo: Verallgemeinern
        Mission baseMission = io::MissionReader::fromFile(getRootDir() + "/test/data/solution_simulation/sample_mission/test1/specs/solution_analysis_test_mission.xml", organizationModel);
        solution = SpaceTime::Network::fromFile(getRootDir() + "/test/data/solution_simulation/sample_mission/test1/0/final_solution_network.gexf");
        mission = make_shared<Mission>(baseMission);
        resourceRequirements = Mission::getResourceRequirements(mission);
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
        timeAssignment = utils::quantifyTime(mission, solution, resourceRequirements);
    }

    Mission::Ptr mission;
    OrganizationModel::Ptr organizationModel;
    OrganizationModelAsk omAsk;
    SpaceTime::Network solution;
    std::map<SpaceTime::Network::tuple_t::Ptr, solvers::FluentTimeResource::List> tupleFtrMap;
    solvers::temporal::TemporalConstraintNetwork::Assignment timeAssignment;
    std::vector<solvers::FluentTimeResource> resourceRequirements;

};
struct SimulationFixture2
{
    SimulationFixture2()
    {        
        organizationModel = OrganizationModel::getInstance(getRootDir() + "/test/data/solution_simulation/organization_model/om-project-transterra.owl");
        // todo: Verallgemeinern
        Mission baseMission = io::MissionReader::fromFile(getRootDir() + "/test/data/solution_simulation/sample_mission/test5/specs/mission.xml", organizationModel);
        solution = SpaceTime::Network::fromFile(getRootDir() + "/test/data/solution_simulation/sample_mission/test5/2/final_solution_network.gexf");
        mission = make_shared<Mission>(baseMission);
        resourceRequirements = Mission::getResourceRequirements(mission);
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
        timeAssignment = utils::quantifyTime(mission, solution, resourceRequirements);
    }

    Mission::Ptr mission;
    OrganizationModel::Ptr organizationModel;
    OrganizationModelAsk omAsk;
    SpaceTime::Network solution;
    std::map<SpaceTime::Network::tuple_t::Ptr, solvers::FluentTimeResource::List> tupleFtrMap;
    solvers::temporal::TemporalConstraintNetwork::Assignment timeAssignment;
    std::vector<solvers::FluentTimeResource> resourceRequirements;

};

struct SimulationFixture3
{
    SimulationFixture3()
    {        
        organizationModel = OrganizationModel::getInstance(getRootDir() + "/test/data/solution_simulation/organization_model/om-project-transterra.owl");
        // todo: Verallgemeinern
        Mission baseMission = io::MissionReader::fromFile(getRootDir() + "/test/data/solution_simulation/sample_mission/test2/specs/4_mission.xml", organizationModel);
        solution = SpaceTime::Network::fromFile(getRootDir() + "/test/data/solution_simulation/sample_mission/test2/4/final_solution_network.gexf");
        mission = make_shared<Mission>(baseMission);
        resourceRequirements = Mission::getResourceRequirements(mission);
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
        timeAssignment = utils::quantifyTime(mission, solution, resourceRequirements);
    }

    Mission::Ptr mission;
    OrganizationModel::Ptr organizationModel;
    OrganizationModelAsk omAsk;
    SpaceTime::Network solution;
    std::map<SpaceTime::Network::tuple_t::Ptr, solvers::FluentTimeResource::List> tupleFtrMap;
    solvers::temporal::TemporalConstraintNetwork::Assignment timeAssignment;
    std::vector<solvers::FluentTimeResource> resourceRequirements;

};


BOOST_FIXTURE_TEST_CASE(test_simulation1, SimulationFixture1)
{
    double n = 10000;
    std::vector<utils::ProbabilityType> metricsList;
    metricsList.push_back(utils::POF);
    solvers::SolutionSimulation simulator(n, metricsList, 0.7);
    double count = 0;
    BOOST_TEST_MESSAGE("Starting simulation for " << n << " runs,");
    for (int i=0; i < n; ++i)
    {
        try
        {
            if (simulator.run(mission, solution, mission->getOrganizationModelAsk(), tupleFtrMap, timeAssignment, resourceRequirements, false)) ++count;
        }
        catch(const std::exception& e)
        {
            std::cout << "whyy?" << std::endl;
        }
    }
    solvers::ResultAnalysis resultAnalysis = simulator.analyzeSimulationResults();
    CSVLogger csvLoggerEfficacyTriple({"timepoint", "failedComponents", "avgEfficacy", "minEfficacy", "maxEfficacy"});
    
    for (size_t i = 0; i < resultAnalysis.failedToEfficacyTripleList.size(); i++)
        {
            csvLoggerEfficacyTriple.addToRow(i, "timepoint");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].first, "failedComponents");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.avg, "avgEfficacy");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.min, "minEfficacy");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.max, "maxEfficacy");
            csvLoggerEfficacyTriple.commitRow();
        }
    csvLoggerEfficacyTriple.save("/tmp/sim_result_1_failedToEfficacyTripleResult.log");

    CSVLogger csvEfficacies({"efficacy", "count"});
    for (auto &e : resultAnalysis.efficacyCounts)
    {
        csvEfficacies.addToRow(e.first, "efficacy");
        csvEfficacies.addToRow(e.second, "count");
        csvEfficacies.commitRow();
    }
    csvEfficacies.save("/tmp/sim_result_1_efficacyCountsResult.log");
    std::string filename("/tmp/sim_result_1_componentFailures.log");
    std::ofstream outfile(filename, std::ofstream::out);
    outfile << "# component count " << std::endl;
    for (auto &a : resultAnalysis.individualComponentFailureCountList)
    {
        std::cout << "Component: " << a.first.getName().toString() << " failed " << a.second << " times out of " << n << " runs." << std::endl;
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
    csvComponentFailuresToEfficacy.save("/tmp/sim_result_1_componentFailuresToEfficacy.log");

    std::ofstream importanceFactorOut("/tmp/sim_result_1_importanceFactors.log", std::ofstream::out);
    importanceFactorOut << "# component importanceFactor " << std::endl;
    for (auto &l : resultAnalysis.componentImportanceFactors)
    {
        importanceFactorOut << l.first << " " << l.second << std::endl;
    }
    importanceFactorOut.close();
    double success_rate = count / n;
    std::ofstream statsOut("/tmp/sim_result_1_stats.log", std::ofstream::out);
    statsOut << "Efficacy: " << resultAnalysis.avgEfficacy;
    statsOut.close();
    BOOST_TEST_MESSAGE("From " << n << " simulation runs, " << count << " succeeded. Success rate was: " << success_rate << ".");
}

BOOST_FIXTURE_TEST_CASE(test_simulation2, SimulationFixture2)
{
    double n = 10000;
    std::vector<utils::ProbabilityType> metricsList;
    metricsList.push_back(utils::POF);
    solvers::SolutionSimulation simulator(n, metricsList, 0.7);
    double count = 0;
    BOOST_TEST_MESSAGE("Starting simulation for " << n << " runs,");
    for (int i=0; i < n; ++i)
    {
        std::cout << "Starting Simulation run " << i+1 << std::endl;
        try
        {
            if (simulator.run(mission, solution, mission->getOrganizationModelAsk(), tupleFtrMap, timeAssignment, resourceRequirements, false)) ++count;
        }
        catch(const std::exception& e)
        {
            std::cout << "whyy?" << std::endl;
        }
    }
    solvers::ResultAnalysis resultAnalysis = simulator.analyzeSimulationResults();
    CSVLogger csvLoggerEfficacyTriple({"timepoint", "failedComponents", "avgEfficacy", "minEfficacy", "maxEfficacy"});
    
    for (size_t i = 0; i < resultAnalysis.failedToEfficacyTripleList.size(); i++)
        {
            csvLoggerEfficacyTriple.addToRow(i, "timepoint");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].first, "failedComponents");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.avg, "avgEfficacy");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.min, "minEfficacy");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.max, "maxEfficacy");
            csvLoggerEfficacyTriple.commitRow();
        }
    csvLoggerEfficacyTriple.save("/tmp/sim_result_2_failedToEfficacyTripleResult.log");

    CSVLogger csvEfficacies({"efficacy", "count"});
    for (auto &e : resultAnalysis.efficacyCounts)
    {
        csvEfficacies.addToRow(e.first, "efficacy");
        csvEfficacies.addToRow(e.second, "count");
        csvEfficacies.commitRow();
    }
    csvEfficacies.save("/tmp/sim_result_2_efficacyCountsResult.log");
    std::string filename("/tmp/sim_result_2_componentFailures.log");
    std::ofstream outfile(filename, std::ofstream::out);
    outfile << "# component count " << std::endl;
    for (auto &a : resultAnalysis.individualComponentFailureCountList)
    {
        std::cout << "Component: " << a.first.getName().toString() << " failed " << a.second << " times out of " << n << " runs." << std::endl;
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
    csvComponentFailuresToEfficacy.save("/tmp/sim_result_2_componentFailuresToEfficacy.log");

    std::ofstream importanceFactorOut("/tmp/sim_result_2_importanceFactors.log", std::ofstream::out);
    importanceFactorOut << "# component importanceFactor " << std::endl;
    for (auto &l : resultAnalysis.componentImportanceFactors)
    {
        importanceFactorOut << l.first << " " << l.second << std::endl;
    }
    importanceFactorOut.close();
    double success_rate = count / n;
    std::ofstream statsOut("/tmp/sim_result_2_stats.log", std::ofstream::out);
    statsOut << "Efficacy: " << resultAnalysis.avgEfficacy;
    statsOut.close();
    BOOST_TEST_MESSAGE("From " << n << " simulation runs, " << count << " succeeded. Success rate was: " << success_rate << ".");
}

BOOST_FIXTURE_TEST_CASE(test_simulation3, SimulationFixture3)
{
    double n = 10000;
    std::vector<utils::ProbabilityType> metricsList;
    metricsList.push_back(utils::POF);
    solvers::SolutionSimulation simulator(n, metricsList, 0.7);
    double count = 0;
    BOOST_TEST_MESSAGE("Starting simulation for " << n << " runs,");
    for (int i=0; i < n; ++i)
    {
        std::cout << "Starting Simulation run " << i+1 << std::endl;
        try
        {
            if (simulator.run(mission, solution, mission->getOrganizationModelAsk(), tupleFtrMap, timeAssignment, resourceRequirements, false)) ++count;
        }
        catch(const std::exception& e)
        {
            std::cout << "whyy?" << std::endl;
        }
    }
    solvers::ResultAnalysis resultAnalysis = simulator.analyzeSimulationResults();
    CSVLogger csvLoggerEfficacyTriple({"timepoint", "failedComponents", "avgEfficacy", "minEfficacy", "maxEfficacy"});
    
    for (size_t i = 0; i < resultAnalysis.failedToEfficacyTripleList.size(); i++)
        {
            csvLoggerEfficacyTriple.addToRow(i, "timepoint");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].first, "failedComponents");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.avg, "avgEfficacy");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.min, "minEfficacy");
            csvLoggerEfficacyTriple.addToRow(resultAnalysis.failedToEfficacyTripleList[i].second.max, "maxEfficacy");
            csvLoggerEfficacyTriple.commitRow();
        }
    csvLoggerEfficacyTriple.save("/tmp/sim_result_3_failedToEfficacyTripleResult.log");

    CSVLogger csvEfficacies({"efficacy", "count"});
    for (auto &e : resultAnalysis.efficacyCounts)
    {
        csvEfficacies.addToRow(e.first, "efficacy");
        csvEfficacies.addToRow(e.second, "count");
        csvEfficacies.commitRow();
    }
    csvEfficacies.save("/tmp/sim_result_3_efficacyCountsResult.log");
    std::string filename("/tmp/sim_result_3_componentFailures.log");
    std::ofstream outfile(filename, std::ofstream::out);
    outfile << "# component count " << std::endl;
    for (auto &a : resultAnalysis.individualComponentFailureCountList)
    {
        std::cout << "Component: " << a.first.getName().toString() << " failed " << a.second << " times out of " << n << " runs." << std::endl;
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
    csvComponentFailuresToEfficacy.save("/tmp/sim_result_3_componentFailuresToEfficacy.log");

    std::ofstream importanceFactorOut("/tmp/sim_result_3_importanceFactors.log", std::ofstream::out);
    importanceFactorOut << "# component importanceFactor " << std::endl;
    for (auto &l : resultAnalysis.componentImportanceFactors)
    {
        importanceFactorOut << l.first << " " << l.second << std::endl;
    }
    importanceFactorOut.close();

    std::ofstream statsOut("/tmp/sim_result_3_stats.log", std::ofstream::out);
    statsOut << "Efficacy: " << resultAnalysis.avgEfficacy;
    statsOut.close();
    double success_rate = count / n;
    BOOST_TEST_MESSAGE("From " << n << " simulation runs, " << count << " succeeded. Success rate was: " << success_rate << ". " << "Efficacy was: " << resultAnalysis.avgEfficacy);
}