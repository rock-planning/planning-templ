// Do NOT add anything to this file
// This header from boost takes ages to compile, so we make sure it is compiled
// only once (here)
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include "test_utils.hpp"

std::string getRootDir()
{
    char buffer[1024];
    BOOST_REQUIRE_MESSAGE( readlink("/proc/self/exe", buffer, 1024) != -1, "Retrieving current execution path");
    std::string str(buffer);
    std::string executionDir = str.substr(0, str.rfind("templ/"));
    std::string configurationPath = executionDir + "templ/";
    return configurationPath;
}
