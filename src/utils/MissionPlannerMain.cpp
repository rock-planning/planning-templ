#include <templ/io/MissionReader.hpp>
#include <templ/MissionPlanner.hpp>
#include <boost/program_options.hpp>

using namespace templ;

int main(int argc, char** argv)
{

    namespace po = boost::program_options;

    po::options_description description("allowed options");
    description.add_options()
        ("help","describe arguments")
        ("mission", po::value<std::string>(), "Path to the mission specification")
        ("om", po::value<std::string>(), "IRI of the organization model (optional)")
        ("min_solutions", po::value<size_t>(), "Minimum number of solutions (optional)")
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
    if(vm.count("solutions"))
    {
        minimumNumberOfSolutions = vm["solutions"].as<size_t>();
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

    organization_model::OrganizationModel::Ptr organizationModel;
    if(vm.count("om"))
    {
        owlapi::model::IRI organizationModelFilename(vm["om"].as<std::string>());
        organizationModel = organization_model::OrganizationModel::getInstance(organizationModelFilename);
    }

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
            Plan::saveAsActionPlan(solutions, filename);
        }
    }

    return 0;
}
