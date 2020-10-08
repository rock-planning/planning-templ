#include "../solvers/SolutionAnalysis.hpp"
#include "../io/MissionReader.hpp"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>

using namespace templ;
namespace po = boost::program_options;

void processFile(const Mission::Ptr& mission, const std::string& solutionFilename, const po::variables_map& vm, int session_id = -1)
{
    std::cout << "Processing: " << solutionFilename << std::endl;
    SpaceTime::Network solution = SpaceTime::Network::fromFile(solutionFilename, mission->getLocations(), mission->getTimepoints());

    solvers::SolutionAnalysis solutionAnalysis(mission, solution);
    solutionAnalysis.analyse();

    std::string saveFilename;
    if(vm.count("save"))
    {
        saveFilename = vm["save"].as<std::string>();
        solutionAnalysis.save(saveFilename);
    }

    if(vm.count("save-row"))
    {
        saveFilename = vm["save-row"].as<std::string>();
        size_t sessionId = 0;

        if(session_id > -1)
        {
            sessionId = session_id;
        } else {
            if(vm.count("session-id"))
            {
                sessionId = vm["session-id"].as<size_t>();
            }
        }
        solutionAnalysis.saveRow(saveFilename, sessionId);
    }

    if(vm.count("save-modelpool"))
    {
        saveFilename = vm["save-modelpool"].as<std::string>();
        solutionAnalysis.saveModelPool(saveFilename);
    }

    if(vm.count("report"))
    {
        printf("Report:\n%s", solutionAnalysis.toString().c_str());
    }
}

void processDir(const Mission::Ptr& mission, const std::string& solutionDir, const po::variables_map& vm)
{
    using namespace boost::filesystem;
    path logPath(solutionDir);
    if(!is_directory(logPath))
    {
        printf("Given log-dir does not exist!");
        exit(2);
    }

    std::vector<path> paths;
    for(const auto& dir : directory_iterator(logPath))
    {
        if(is_directory(dir))
        {
            try {
                std::stoi(dir.path().stem().string());
                paths.push_back(dir.path());
            } catch(std::invalid_argument& e)
            {
                std::cout << "Ignoring folder: " << dir << std::endl;;
            }
        }
    }

    std::sort(paths.begin(), paths.end(), [](const path& a, const path& b)
            {
                int a_value = std::stoi(a.stem().string());
                int b_value = std::stoi(b.stem().string());
                return a_value < b_value;
            });

    for(const auto& dir : paths)
    {
        path solution_path = dir / "final_solution_network.gexf";
        int session_id = stoi(dir.stem().string());
        processFile(mission, solution_path.string(), vm, session_id);
    }
}

int main(int argc, char** argv)
{
    po::options_description description("allowed options");
    description.add_options()
        ("help","describe arguments")
        ("mission", po::value<std::string>(), "Path to the mission specification")
        ("log-dir", po::value<std::string>(), "Path to the logdirectory containing the  solution files")
        ("solution", po::value<std::string>(), "Path to the solution file")
        ("om", po::value<std::string>(), "IRI of the organization model (optional)")
        ("report", "show the report of the analysis")
        ("save", po::value<std::string>(), "Save final path to a given filename")
        ("save-row", po::value<std::string>(), "Save generated row data to a given filename")
        ("save-modelpool", po::value<std::string>(), "Save generated modelpool to a given filename")
        ("session-id", po::value<size_t>(), "The session id to use for writing row data")
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

    moreorg::OrganizationModel::Ptr organizationModel;
    if(vm.count("om"))
    {
        owlapi::model::IRI organizationModelFilename(vm["om"].as<std::string>());
        organizationModel = moreorg::OrganizationModel::getInstance(organizationModelFilename);
    }

    std::string solutionFilename;
    if(vm.count("solution") && vm.count("log-dir"))
    {
        printf("You cannot use solution and log-dir options at the same time");
        exit(3);
    }

    using namespace templ;
    Mission baseMission = io::MissionReader::fromFile(missionFilename, organizationModel);

    Mission::Ptr mission = make_shared<Mission>(baseMission);

    if(vm.count("solution"))
    {
        processFile(mission, vm["solution"].as<std::string>(), vm);
    } else if(vm.count("log-dir"))
    {
        processDir(mission, vm["log-dir"].as<std::string>(), vm);
    } else
    {
        printf("Please provide at least a solution to start the analysis\n");
        exit(2);
    }

    return 0;
}


