#include <templ/io/MissionReader.hpp>
//#include <templ/MissionPlanner.hpp>

#include <templ/solvers/csp/RoleDistribution.hpp>

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        printf("usage: %s <mission> <organization-model>\n", argv[0]);
        exit(-1);
    }

    std::string missionFilename = argv[1];
    std::string organizationModelFilename = argv[2];

    using namespace templ;
    Mission mission = io::MissionReader::fromFile(missionFilename);
    printf("%s\n",mission.toString().c_str());

    using namespace organization_model;
    OrganizationModel::Ptr organizationModel = OrganizationModel::getInstance(organizationModelFilename);
    mission.setOrganizationModel(organizationModel);


    std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
    if(!solutions.empty())
    {
        std::cout << "Solutions for model distribution found: " << solutions << std::endl;
    }

    // Check role distribution after model distribution
    std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0]);
    if(!roleSolutions.empty())
    {
        std::cout << "Solutions for role distribution: " << roleSolutions << std::endl;
    }


//    MissionPlanner missionPlanner(mission);
//    CandidateMissions missions = missionPlanner.solve();

    return 0;
}
