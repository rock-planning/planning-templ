#include <boost/program_options.hpp>
#include <graph_analysis/GraphIO.hpp>

#include "../io/MissionReader.hpp"
#include "../solvers/csp/TransportNetwork.hpp"
#include "../solvers/SolutionAnalysis.hpp"

using namespace templ;

int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description description("allowed options");
    description.add_options()
        ("help","describe arguments")
        ("mission", po::value<std::string>(), "Path to the mission specification")
        ("configuration", po::value<std::string>(), "Path to the search configuration file")
        ("om", po::value<std::string>(), "IRI of the organization model (optional)")
        ("min_solutions", po::value<size_t>(), "Minimum number of solutions (optional)")
        ("interactive", po::value<bool>(), "Set interactive mode (optional, otherwise setting from configuration file is used)")
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

    organization_model::OrganizationModel::Ptr organizationModel;
    if(vm.count("om"))
    {
        owlapi::model::IRI organizationModelFilename(vm["om"].as<std::string>());
        organizationModel = organization_model::OrganizationModel::getInstance(organizationModelFilename);
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
    configuration.save(mission->getLogger()->getBasePath() + "/specs/configuration.xml");

    // Make sure the configuration is set according to the command line
    // parameter if selected, otherwise use the info from the configuration file
    if(vm.count("interactive"))
    {
        bool interactive = vm["interactive"].as<bool>();
        if(interactive)
        {
            configuration.setValue("TransportNetwork/search/interactive", "true");
        } else {
            configuration.setValue("TransportNetwork/search/interactive", "false");
        }
    }


    std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,minimumNumberOfSolutions, configuration);

    std::cout << "TemPl:" << std::endl;
    std::cout << "    # of solutions found: " << solutions.size()
        << std::endl
        << "    Check log directory: " << mission->getLogger()->getBasePath()
        << std::endl;

    return 0;
}
