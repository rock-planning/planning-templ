#include <templ/io/MissionReader.hpp>
#include <templ/MissionPlanner.hpp>

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
    Mission mission = io::MissionReader::fromFile(missionFilename, organizationModel);
    printf("%s\n",mission.toString().c_str());

    MissionPlanner missionPlanner(mission);

    std::vector<Plan> solutions = missionPlanner.execute(minimumNumberOfSolutions);
    if(solutions.empty())
    {
        std::cout << "No solution found" << std::endl;
    } else {
        std::cout << solutions.size() << " solutions have been found: " << Plan::toString(solutions) << std::endl;

        mission.getLogger()->disableSessions();
        {
            std::string filename = mission.getLogger()->filename("plans.log");
            Plan::save(solutions, filename);
        }
        {
            std::string filename = mission.getLogger()->filename("actionplans.log");
            Plan::saveAsActionPlan(solutions, mission, filename);
        }
    }

    return 0;
}
