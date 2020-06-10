#include <graph_analysis/GraphIO.hpp>
#include <boost/program_options.hpp>

#include "../io/MissionReader.hpp"
#include "../solvers/SolutionAnalysis.hpp"
#include "../solvers/Solver.hpp"

using namespace templ;

int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description description("allowed options");
    description.add_options()
        ("help","describe arguments")
        ("solver", po::value<std::string>(), "Solver name")
        ("mission", po::value<std::string>(), "Path to the mission specification")
        ("configuration", po::value<std::string>(), "Path to the search configuration file")
        ("om", po::value<std::string>(), "IRI of the organization model (optional)")
        ("min_solutions", po::value<size_t>(), "Minimum number of solutions (optional)")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
        std::cout << description << std::endl;
        exit(1);
    }

    size_t minimumNumberOfSolutions = 1;
    if(vm.count("min_solutions"))
    {
        minimumNumberOfSolutions = vm["min_solutions"].as<size_t>();
        std::cout << "Minimum number of requested solutions: " << minimumNumberOfSolutions << std::endl;
    }

    std::string missionFilename;
    if(vm.count("mission"))
    {
        missionFilename = vm["mission"].as<std::string>();
    } else {
        printf("Please provide at least a mission to start the planning process\n");
        exit(2);
    }
    std::string configurationFilename;
    if(vm.count("configuration"))
    {
        configurationFilename = vm["configuration"].as<std::string>();
    }

    moreorg::OrganizationModel::Ptr organizationModel;
    if(vm.count("om"))
    {
        owlapi::model::IRI organizationModelFilename(vm["om"].as<std::string>());
        organizationModel = moreorg::OrganizationModel::getInstance(organizationModelFilename);
    }

    solvers::Solver::SolverType solverType = solvers::Solver::UNKNOWN;
    if(vm.count("solver"))
    {
        for(const std::pair<solvers::Solver::SolverType, std::string>& t : solvers::Solver::SolverTypeTxt)
        {
            if(t.second == vm["solver"].as<std::string>())
            {
                solverType = t.first;
                break;
            }
        }
    } else {
        std::cout << "Solver name required: one of" << std::endl;
        for(const std::pair<solvers::Solver::SolverType, std::string>& t : solvers::Solver::SolverTypeTxt)
        {
            std::cout << t.second << "," << std::endl;
        }
    }

    using namespace templ;
    using namespace templ;
    Mission baseMission = io::MissionReader::fromFile(missionFilename, organizationModel);
    baseMission.prepareTimeIntervals();
    baseMission.applyOrganizationModelOverrides();

    Mission::Ptr mission(new Mission(baseMission));

    mission->saveInputData(mission->getLogger()->getBasePath()  + "/specs");
    std::string dotFilename = mission->getLogger()->filename("templ-mission-relations.dot");
    graph_analysis::io::GraphIO::write(dotFilename, mission->getRelations());
    printf("Written: %s\n", dotFilename.c_str() );
    printf("Base Mission:\n %s\n",mission->toString().c_str());


    qxcfg::Configuration configuration(configurationFilename);
    solvers::Solver::Ptr solver = solvers::Solver::getInstance(solverType);
    solvers::Session::Ptr session = solver->run(mission,minimumNumberOfSolutions, configuration);
    solvers::Solution::List solutions = session->getSolutions();
    if(solutions.empty())
    {
        std::cout << "No solution found" << std::endl;
    }

    return 0;
}
