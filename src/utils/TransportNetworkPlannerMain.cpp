#include <templ/io/MissionReader.hpp>
#include <templ/solvers/csp/TransportNetwork.hpp>
#include <graph_analysis/GraphIO.hpp>
#include "../solvers/SolutionAnalysis.hpp"
#include <boost/program_options.hpp>

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
    if(solutions.empty())
    {
        std::cout << "No solution found" << std::endl;
    //} else {
    //    std::cout << solutions.size() << " solutions have been found" << std::endl; // << solutions;

    //    for(size_t i = 0; i < solutions.size(); ++i)
    //    {
    //        std::string filename;
    //        try {
    //            std::stringstream ss;
    //            ss << "templ-mission-solution-" << i << ".dot";
    //            filename = mission->getLogger()->filename(ss.str());
    //            graph_analysis::io::GraphIO::write(filename, solutions[i].getMinCostFlowSolution().getGraph());
    //        } catch(const std::exception& e)
    //        {
    //            std::cout << "Saving file " << filename << " failed: -- " << e.what();
    //        }

    //        try {
    //            std::stringstream ss;
    //            ss << "templ-mission-solution-" << i << ".gexf";
    //            filename = mission->getLogger()->filename(ss.str());
    //            solutions[i].getMinCostFlowSolution().save(filename, "gexf");
    //        } catch(const std::exception& e)
    //        {
    //            std::cout << "Saving file " << filename << " failed: -- " << e.what();
    //        }


    //        std::cout << "Solution analysis" << std::endl;
    //        solvers::SolutionAnalysis sa(mission, solutions[i].getMinCostFlowSolution());
    //        std::cout << "    Required roles: " << Role::toString(sa.getRequiredRoles()) << std::endl;

    //        for(const symbols::constants::Location::Ptr& location : mission->getLocations())
    //        {
    //            for(const solvers::temporal::Interval& interval : mission->getTimeIntervals())
    //            {

    //                std::cout << "At location: " << location->toString() << std::endl;
    //                std::cout << "     " << interval.toString() << std::endl;

    //                std::vector<organization_model::ModelPool> pools = sa.getAvailableResources(location, interval);

    //                for(const organization_model::ModelPool& pool : pools)
    //                {
    //                    std::cout << pool.toString(8) << std::endl;
    //                }
    //            }
    //        }

    //        graph_analysis::BaseGraph::Ptr hyperGraph = sa.toHyperGraph();
    //        try {
    //            std::stringstream ss;
    //            ss << "templ-mission-solution-hypergraph" << i << ".dot";
    //            filename = mission->getLogger()->filename(ss.str());
    //            graph_analysis::io::GraphIO::write(filename, hyperGraph);
    //        } catch(const std::exception& e)
    //        {
    //            std::cout << "Saving file " << filename << " failed: -- " << e.what();
    //        }

    //    }

    //    mission->getLogger()->disableSessions();
    }

    return 0;
}
