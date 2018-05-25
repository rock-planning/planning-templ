#include "../io/MissionReader.hpp"
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
        ("file", po::value<std::string>(), "Path to the mission specification")
        ("om", po::value<std::string>(), "IRI of the organization model (optional)")
        ("latex", po::value<std::string>(), "write as latex code")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);
    po::notify(vm);

    if(vm.count("help") || !vm.count("file"))
    {
        std::cout << description << std::endl;
        exit(1);
    }

    std::string filename = vm["file"].as<std::string>();

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



    std::string dotFilename = "/tmp/templ-mission-relations.dot";
    graph_analysis::io::GraphIO::write(dotFilename, mission->getRelations());
    dotFilename = "Written '" + dotFilename + "'";
    std::cout << dotFilename << std::endl;

    return 0;
}
