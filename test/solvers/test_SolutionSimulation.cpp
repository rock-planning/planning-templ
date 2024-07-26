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
    }

    Mission::Ptr mission;
    OrganizationModel::Ptr organizationModel;
    OrganizationModelAsk omAsk;
    SpaceTime::Network solution;

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
    }

    Mission::Ptr mission;
    OrganizationModel::Ptr organizationModel;
    SpaceTime::Network solution;

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
    }

    Mission::Ptr mission;
    OrganizationModel::Ptr organizationModel;
    OrganizationModelAsk omAsk;
    SpaceTime::Network solution;
};


BOOST_AUTO_TEST_SUITE(solution_simulation)

BOOST_FIXTURE_TEST_CASE(simulation_1, SimulationFixture1)
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
            if (simulator.run(mission, solution, false)) ++count;
        }
        catch(const std::exception& e)
        {
            std::cout << "whyy?" << std::endl;
        }
    }
    solvers::ResultAnalysis resultAnalysis = simulator.analyzeSimulationResults();
    simulator.saveSimulationResults("/tmp/sim_result_1_");
    double success_rate = count / n;
    BOOST_TEST_MESSAGE("From " << n << " simulation runs, " << count << " succeeded. Success rate was: " << success_rate << ".");
}

BOOST_FIXTURE_TEST_CASE(simulation_2, SimulationFixture2)
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
            if (simulator.run(mission, solution, false)) ++count;
        }
        catch(const std::exception& e)
        {
            std::cout << "whyy?" << std::endl;
        }
    }
    solvers::ResultAnalysis resultAnalysis = simulator.analyzeSimulationResults();
    simulator.saveSimulationResults("/tmp/sim_result_2_");
    double success_rate = count / n;
    BOOST_TEST_MESSAGE("From " << n << " simulation runs, " << count << " succeeded. Success rate was: " << success_rate << ".");
}

BOOST_FIXTURE_TEST_CASE(simulation_3, SimulationFixture3)
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
            if (simulator.run(mission, solution, false)) ++count;
        }
        catch(const std::exception& e)
        {
            std::cout << "whyy?" << std::endl;
        }
    }
    solvers::ResultAnalysis resultAnalysis = simulator.analyzeSimulationResults();
    simulator.saveSimulationResults("/tmp/sim_result_3_");
    double success_rate = count / n;
    BOOST_TEST_MESSAGE("From " << n << " simulation runs, " << count << " succeeded. Success rate was: " << success_rate << ". " << "Efficacy was: " << resultAnalysis.avgEfficacy);
}

BOOST_AUTO_TEST_SUITE_END()
