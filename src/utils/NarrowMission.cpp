#include "../io/MissionReader.hpp"
#include "../io/MissionWriter.hpp"
#include "../io/LatexWriter.hpp"
#include "../solvers/FluentTimeResource.hpp"
#include "../solvers/Solution.hpp"

#include <graph_analysis/GraphIO.hpp>
#include <boost/program_options.hpp>
#include <iostream>

using namespace templ;

/// Simple reader utility
int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description description("allowed options");
    description.add_options()
        ("help","describe arguments")
        ("mission", po::value<std::string>(), "Path to the mission specification")
        ("solution", po::value<std::string>(), "Mission solution that shall be used for narrowing")
        ("out",po::value<std::string>(), "write mission with mission writer")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);
    po::notify(vm);

    if(vm.count("help") || !vm.count("mission"))
    {
        std::cout << description << std::endl;
        exit(1);
    }

    if(!vm.count("mission"))
    {
        std::cout << "Please provide a path to a mission" << std::endl;
        std::cout << description << std::endl;
        exit(2);
    }

    std::string filename = vm["mission"].as<std::string>();
    Mission::Ptr mission = make_shared<Mission>(io::MissionReader::fromFile(filename));

    if(!vm.count("solution"))
    {
        std::cout << "Please provide a path to a solution" << std::endl;
        std::cout << description << std::endl;
        exit(3);
    }

    std::string solutionFilename = vm["solution"].as<std::string>();
    solvers::Solution loadedSolution = solvers::Solution::fromFile(solutionFilename, mission->getOrganizationModel());

    Mission::Ptr narrowedMission = loadedSolution.getNarrowedMission(*mission);

    if(vm.count("out"))
    {
        std::string outfile = vm["out"].as<std::string>();
        io::MissionWriter::write(outfile, *narrowedMission);
    }

    return 0;
}
