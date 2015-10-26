#include <templ/io/MissionReader.hpp>
#include <templ/MissionPlanner.hpp>

using namespace templ;

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
    
    MissionPlanner missionPlanner(mission, organizationModel);
    missionPlanner.execute();
    missionPlanner.save();

    return 0;
}
