#include <boost/test/unit_test.hpp>
#include "test_utils.hpp"

#include <templ/benchmark/io/GoldenReader.hpp>

using namespace templ::benchmark;

BOOST_AUTO_TEST_SUITE(benchmark)

BOOST_AUTO_TEST_CASE(golden_io)
{
    std::string goldenFile = getRootDir() + "/test/data/benchmarks/golden/Golden_5.vrp";

    io::GoldenReader reader;
    VRPProblem vrp = reader.read(goldenFile);
    BOOST_REQUIRE_MESSAGE(vrp.getName() == "Golden_5", "Name of benchmark found");
    BOOST_REQUIRE_MESSAGE(vrp.getNodeCoordinates().size() == 200, "200 node coordinates found, found: " << vrp.getNodeCoordinates().size());
    BOOST_TEST_MESSAGE(vrp.toString(8));

}

BOOST_AUTO_TEST_CASE(uchoa_io)
{
    std::string goldenFile = getRootDir() + "/test/data/benchmarks/X/X-n1001-k43.vrp";

    io::GoldenReader reader;
    VRPProblem vrp = reader.read(goldenFile);
    BOOST_REQUIRE_MESSAGE(vrp.getNodeCoordinates().size() == 1001, "1001 node coordinates found, found: " << vrp.getNodeCoordinates().size());
    BOOST_TEST_MESSAGE(vrp.toString(8));

}

BOOST_AUTO_TEST_SUITE_END()
