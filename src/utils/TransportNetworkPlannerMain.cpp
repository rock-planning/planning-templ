#include <templ/io/MissionReader.hpp>
#include <templ/solvers/csp/TransportNetwork.hpp>
#include <graph_analysis/GraphIO.hpp>
#include "../solvers/SolutionAnalysis.hpp"

using namespace templ;

int main(int argc, char** argv)
{
    if(argc < 3)
    {
        printf("usage: %s <mission> <organization-model> [<min-number-of-solutions>]\n", argv[0]);
        exit(-1);
    }

    int minimumNumberOfSolutions = 1;
    if(argc == 4)
    {
        minimumNumberOfSolutions = atoi(argv[3]);
        std::cout << "Minimum number of requested solutions: " << minimumNumberOfSolutions << std::endl;
    }

    std::string missionFilename = argv[1];
    std::string organizationModelFilename = argv[2];
    organization_model::OrganizationModel::Ptr organizationModel = organization_model::OrganizationModel::getInstance(organizationModelFilename);

    using namespace templ;
    Mission baseMission = io::MissionReader::fromFile(missionFilename, organizationModel);
    baseMission.prepareTimeIntervals();

    Mission::Ptr mission(new Mission(baseMission));

    mission->getLogger()->saveInputData(*mission.get());
    std::string dotFilename = mission->getLogger()->filename("templ-mission-relations.dot");
    graph_analysis::io::GraphIO::write(dotFilename, mission->getRelations());
    printf("Written: %s\n", dotFilename.c_str() );
    printf("Base Mission:\n %s\n",mission->toString().c_str());


    std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,minimumNumberOfSolutions);
    if(solutions.empty())
    {
        std::cout << "No solution found" << std::endl;
    } else {
        std::cout << solutions.size() << " solutions have been found" << std::endl; // << solutions;

        for(size_t i = 0; i < solutions.size(); ++i)
        {
            std::string filename;
            try {
                std::stringstream ss;
                ss << "templ-mission-solution-" << i << ".dot";
                filename = mission->getLogger()->filename(ss.str());
                graph_analysis::io::GraphIO::write(filename, solutions[i].getMinCostFlowSolution().getGraph());
            } catch(const std::exception& e)
            {
                std::cout << "Saving file " << filename << " failed: -- " << e.what();
            }

            try {
                std::stringstream ss;
                ss << "templ-mission-solution-" << i << ".gexf";
                filename = mission->getLogger()->filename(ss.str());
                solutions[i].getMinCostFlowSolution().save(filename, "gexf");
            } catch(const std::exception& e)
            {
                std::cout << "Saving file " << filename << " failed: -- " << e.what();
            }


            std::cout << "Solution analysis" << std::endl;
            solvers::SolutionAnalysis sa(mission, solutions[i].getMinCostFlowSolution());
            std::cout << "    Required roles: " << Role::toString(sa.getRequiredRoles()) << std::endl;

            for(const symbols::constants::Location::Ptr& location : mission->getLocations())
            {
                for(const solvers::temporal::Interval& interval : mission->getTimeIntervals())
                {

                    std::cout << "At location: " << location->toString() << std::endl;
                    std::cout << "     " << interval.toString() << std::endl;

                    std::vector<organization_model::ModelPool> pools = sa.getAvailableResources(location, interval);

                    for(const organization_model::ModelPool& pool : pools)
                    {
                        std::cout << pool.toString(8) << std::endl;
                    }
                }
            }

            graph_analysis::BaseGraph::Ptr hyperGraph = sa.toHyperGraph();
            try {
                std::stringstream ss;
                ss << "templ-mission-solution-hypergraph" << i << ".dot";
                filename = mission->getLogger()->filename(ss.str());
                graph_analysis::io::GraphIO::write(filename, hyperGraph);
            } catch(const std::exception& e)
            {
                std::cout << "Saving file " << filename << " failed: -- " << e.what();
            }

        }

        mission->getLogger()->disableSessions();
    }

    return 0;
}
