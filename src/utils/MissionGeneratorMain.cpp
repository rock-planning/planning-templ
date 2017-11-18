#include "../benchmark/MissionGenerator.hpp"
#include "../benchmark/io/GoldenReader.hpp"
#include "../io/MissionWriter.hpp"
#include <boost/program_options.hpp>

using namespace templ;

int main(int argc, char**argv)
{
    namespace po = boost::program_options;

    po::options_description description("allowed options");
    description.add_options()
        ("help","describe arguments")
        ("om", po::value<std::string>(), "IRI of the organization model (optional)")
        ("vrp", po::value<std::string>(), "Path to vrp file for conversion")
        ("output", po::value<std::string>(), "Output file to generate -- default is /tmp/mission.xml")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
        std::cout << description << std::endl;
        exit(1);
    }

    if(vm.count("vrp"))
    {
        using namespace templ::benchmark;
        std::string vrpFile = vm["vrp"].as<std::string>();
        benchmark::io::GoldenReader reader;
        VRPProblem vrp = reader.read(vrpFile);

        Mission::Ptr mission = MissionGenerator::convert(vrp);
        std::cout << "Converted mission" << std::endl;

        std::string filename = "/tmp/mission.xml";
        if(vm.count("output"))
        {
            filename = vm["output"].as<std::string>();
        }
        std::cout << "Writing mission" << std::endl;
        templ::io::MissionWriter::write(filename, *mission);
        std::cout << "Written mission to: " << filename << std::endl;
    }
}

