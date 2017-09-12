#include "../solvers/SolutionAnalysis.hpp"
#include "../io/MissionReader.hpp"

using namespace templ;

int main(int argc, char** argv)
{

    if(argc != 4)
    {
        printf("usage: %s <mission> <organization-model> <solution-file>\n", argv[0]);
        exit(-1);
    }

    std::string missionFilename = argv[1];
    std::string organizationModelFilename = argv[2];
    organization_model::OrganizationModel::Ptr organizationModel = organization_model::OrganizationModel::getInstance(organizationModelFilename);

    using namespace templ;
    Mission baseMission = io::MissionReader::fromFile(missionFilename, organizationModel);
    baseMission.prepareTimeIntervals();
    Mission::Ptr mission( new Mission(baseMission));

    std::string solutionFilename = argv[3];
    SpaceTime::Network solution = SpaceTime::Network::fromFile(solutionFilename, mission);

    solvers::SolutionAnalysis solutionAnalysis(mission, solution);
    solutionAnalysis.analyse();
    solutionAnalysis.save();

    printf("Report:\n%s", solutionAnalysis.toString().c_str());

    return 0;
}
