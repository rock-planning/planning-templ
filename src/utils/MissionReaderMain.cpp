#include <templ/io/MissionReader.hpp>

/// Simple reader utility
int main(int argc, char** argv)
{
    if(argc != 3)
    {
        printf("usage: %s <file> <organization_model>\n", argv[0]);
        exit(-1);
    }

    std::string filename = argv[1];
    std::string organizationModelFilename = argv[2];

    organization_model::OrganizationModel::Ptr organizationModel = organization_model::OrganizationModel::getInstance(organizationModelFilename);
    using namespace templ;
    Mission mission = io::MissionReader::fromFile(filename, organizationModel);
    mission.prepareTimeIntervals();

    printf("%s\n",mission.toString().c_str());

    return 0;
}
