#include "../solvers/SolutionAnalysis.hpp"
#include "../io/MissionReader.hpp"
#include <boost/program_options.hpp>

using namespace templ;

int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description description("allowed options");
    description.add_options()
        ("help","describe arguments")
        ("mission", po::value<std::string>(), "Path to the mission specification")
        ("solution", po::value<std::string>(), "Path to the solution file")
        ("om", po::value<std::string>(), "IRI of the organization model (optional)")
        ("save", po::value<std::string>(), "Save final path to a given filename")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
        std::cout << description << std::endl;
        exit(1);
    }

    std::string missionFilename;
    if(vm.count("mission"))
    {
        missionFilename = vm["mission"].as<std::string>();
    } else {
        printf("Please provide at least a mission to start the planning process\n");
        exit(2);
    }

    organization_model::OrganizationModel::Ptr organizationModel;
    if(vm.count("om"))
    {
        owlapi::model::IRI organizationModelFilename(vm["om"].as<std::string>());
        organizationModel = organization_model::OrganizationModel::getInstance(organizationModelFilename);
    }

    std::string solutionFilename;
    if(vm.count("solution"))
    {
        solutionFilename = vm["solution"].as<std::string>();
    } else {
        printf("Please provide at least a solution to start the planning process\n");
        exit(2);
    }

    using namespace templ;
    Mission baseMission = io::MissionReader::fromFile(missionFilename, organizationModel);
    baseMission.prepareTimeIntervals();
    baseMission.applyOrganizationModelOverrides();

    Mission::Ptr mission = make_shared<Mission>(baseMission);


    SpaceTime::Network solution = SpaceTime::Network::fromFile(solutionFilename, mission->getLocations(), mission->getTimepoints());
    solvers::SolutionAnalysis solutionAnalysis(mission, solution);
    solutionAnalysis.analyse();

    std::string saveFilename;
    if(vm.count("save"))
    {
        saveFilename = vm["save"].as<std::string>();
    }

    solutionAnalysis.save(saveFilename);

    printf("Report:\n%s", solutionAnalysis.toString().c_str());

    return 0;
}
