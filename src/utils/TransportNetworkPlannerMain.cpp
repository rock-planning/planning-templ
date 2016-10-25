#include <templ/io/MissionReader.hpp>
#include <templ/solvers/csp/TransportNetwork.hpp>

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

    printf("%s\n",baseMission.toString().c_str());
    Mission::Ptr mission(new Mission(baseMission));

    std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,minimumNumberOfSolutions);
    if(solutions.empty())
    {
        std::cout << "No solution found" << std::endl;
    } else {
        std::cout << solutions.size() << " solutions have been found: " << std::endl << solutions;

        mission->getLogger()->disableSessions();
    }

    return 0;
}
