#include "../io/MissionReader.hpp"
#include "../io/MissionWriter.hpp"
#include "../io/LatexWriter.hpp"
#include "../solvers/FluentTimeResource.hpp"

#include <graph_analysis/GraphIO.hpp>
#include <boost/program_options.hpp>
#include <iostream>

/// Simple reader utility
int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description description("allowed options");
    description.add_options()
        ("help","describe arguments")
        ("mission", po::value<std::string>(), "Path to the mission specification")
        ("om", po::value<std::string>(), "IRI of the organization model (optional)")
        ("latex", po::value<std::string>(), "write as latex code")
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

    std::string filename = vm["mission"].as<std::string>();

    std::string organizationModelFilename;
    if(vm.count("om"))
    {
        organizationModelFilename = vm["om"].as<std::string>();
    }

    using namespace templ;
    Mission::Ptr mission;
    if(organizationModelFilename.empty())
    {
        mission = Mission::Ptr(new Mission(io::MissionReader::fromFile(filename)));
    } else {
        mission = Mission::Ptr(new Mission(io::MissionReader::fromFile(filename, organizationModelFilename)));
    }

    mission->prepareTimeIntervals();
    if(vm.count("latex"))
    {
        std::cout << io::LatexWriter::toLatex(mission);
        return 0;
    }

    std::cout << mission->toString() << std::endl;

    std::vector<solvers::FluentTimeResource> ftrs = Mission::getResourceRequirements(mission);
    std::cout << solvers::FluentTimeResource::toString(ftrs) << std::endl;

    if(vm.count("out"))
    {
        std::string outfile = vm["out"].as<std::string>();
        io::MissionWriter::write(outfile, *mission);
    }

    return 0;
}
