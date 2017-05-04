#include <templ/io/MissionReader.hpp>
#include <templ/solvers/csp/TransportNetwork.hpp>
#include <graph_analysis/GraphIO.hpp>

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

    std::string dotFilename = mission->getLogger()->filename("templ-mission-relations.dot");
    graph_analysis::io::GraphIO::write(dotFilename, mission->getRelations());
    printf("Written: %s\n", dotFilename.c_str() );
    printf("Base Mission:\n %s\n",mission->toString().c_str());


    std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,minimumNumberOfSolutions);
    if(solutions.empty())
    {
        std::cout << "No solution found" << std::endl;
    } else {
        std::cout << solutions.size() << " solutions have been found: " << std::endl << solutions;

        for(size_t i = 0; i < solutions.size(); ++i)
        {
            std::stringstream ss;
            ss << "templ-mission-solution-" << i << ".dot";
            std::string filename = mission->getLogger()->filename(ss.str());
            try {
                graph_analysis::io::GraphIO::write(filename, solutions[i].toNetwork().getGraph());
            } catch(const std::exception& e)
            {
                std::cout << "Saving file " << ss.str() << " failed: -- " << e.what();
            }
        }

        mission->getLogger()->disableSessions();
    }

    return 0;
}
