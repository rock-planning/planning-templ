#include <boost/test/unit_test.hpp>
#include "transport_network/TestSpace.hpp"
#include <gecode/search.hh>

using namespace templ;
BOOST_AUTO_TEST_SUITE(csp_temporally_expanded_graph)

void testPathConstraint(const std::vector<bool>& network, size_t numberOfTimepoints, size_t numberOfFluents, const std::string& msg, bool assert)
{
    using namespace templ::test;
    TestSpace* testSpace = new TestSpace(network, numberOfTimepoints, numberOfFluents);
    Gecode::BAB<TestSpace> searchEngine(testSpace);

    TestSpace* first = searchEngine.next();
    if(assert)
    {
        BOOST_REQUIRE_MESSAGE(first != NULL, msg);
    } else {
        BOOST_REQUIRE_MESSAGE(first == NULL, msg);
    }
    delete first;
}

BOOST_AUTO_TEST_CASE(same_source_timepoint)
{
   std::vector<bool> pathArray;
   size_t numberOfTimepoints = 1;
   size_t numberOfFluents = 1;
   pathArray.push_back(true);
   testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "Same timepoint cannot form a path", false);
}

BOOST_AUTO_TEST_CASE(simple_path)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 2;
    size_t numberOfFluents = 1;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(true);  //0-0 -> 1-0
    pathArray.push_back(false); //1-0 -> 0-0
    pathArray.push_back(false); //1-0 -> 1-0
    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "Simple path", true);
}

BOOST_AUTO_TEST_CASE(backward_in_time)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 2;
    size_t numberOfFluents = 1;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(false);  //0-0 -> 1-0
    pathArray.push_back(true); //1-0 -> 0-0
    pathArray.push_back(false); //1-0 -> 1-0
    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "Backward in time is not allowed", false);
}

BOOST_AUTO_TEST_CASE(same_time_link)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 2;
    size_t numberOfFluents = 1;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(false);  //0-0 -> 1-0
    pathArray.push_back(false); //1-0 -> 0-0
    pathArray.push_back(true); //1-0 -> 1-0
    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "Same time link is not allowed", false);
}

BOOST_AUTO_TEST_CASE(parallel_path)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 2;
    size_t numberOfFluents = 1;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(true);  //0-0 -> 1-0
    pathArray.push_back(false); //1-0 -> 0-0
    pathArray.push_back(true); //1-0 -> 1-0
    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "Parallel paths are not allowed", false);
}

BOOST_AUTO_TEST_CASE(no_existing_path)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 2;
    size_t numberOfFluents = 2;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(false); //0-1 -> 0-0
    pathArray.push_back(false);  //0-0 -> 0-1
    pathArray.push_back(false);  //0-1 -> 0-1

    pathArray.push_back(false); //0-0 -> 1-0
    pathArray.push_back(false); //0-1 -> 1-0
    pathArray.push_back(false);  //0-0 -> 1-1
    pathArray.push_back(false);  //0-1 -> 1-1

    pathArray.push_back(false); //1-0 -> 0-0
    pathArray.push_back(false); //1-1 -> 0-0
    pathArray.push_back(false); //1-0 -> 0-1
    pathArray.push_back(false); //1-1 -> 0-1

    pathArray.push_back(false); //1-0 -> 1-0
    pathArray.push_back(false); //1-1 -> 1-0
    pathArray.push_back(false); //1-0 -> 1-1
    pathArray.push_back(false); //1-1 -> 1-1
    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "No existing path", false);
}

BOOST_AUTO_TEST_CASE(multi_edge_path_parallel_0)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 2;
    size_t numberOfFluents = 2;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(false); //0-1 -> 0-0
    pathArray.push_back(true);  //0-0 -> 0-1
    pathArray.push_back(false);  //0-1 -> 0-1

    pathArray.push_back(true); //0-0 -> 1-0
    pathArray.push_back(false); //0-1 -> 1-0
    pathArray.push_back(false);  //0-0 -> 1-1
    pathArray.push_back(false);  //0-1 -> 1-1

    pathArray.push_back(false); //1-0 -> 0-0
    pathArray.push_back(false); //1-1 -> 0-0
    pathArray.push_back(false); //1-0 -> 0-1
    pathArray.push_back(false); //1-1 -> 0-1

    pathArray.push_back(false); //1-0 -> 1-0
    pathArray.push_back(false); //1-1 -> 1-0
    pathArray.push_back(false); //1-0 -> 1-1
    pathArray.push_back(false); //1-1 -> 1-1
    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "No parallel paths allowed", false);
}

BOOST_AUTO_TEST_CASE(multi_edge_path_parallel_1)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 3;
    size_t numberOfFluents = 2;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(false); //0-0 -> 0-1
    pathArray.push_back(true); //0-0 -> 1-0
    pathArray.push_back(true); //0-0 -> 1-1
    pathArray.push_back(false); //0-0 -> 2-0
    pathArray.push_back(false); //0-0 -> 2-1

    pathArray.push_back(false); //0-1 -> 0-0
    pathArray.push_back(false); //0-1 -> 0-1
    pathArray.push_back(false); //0-1 -> 1-0
    pathArray.push_back(false); //0-1 -> 1-1
    pathArray.push_back(false); //0-1 -> 2-0
    pathArray.push_back(false); //0-1 -> 2-1

    pathArray.push_back(false); //1-0 -> 0-0
    pathArray.push_back(false); //1-0 -> 0-1
    pathArray.push_back(false); //1-0 -> 1-0
    pathArray.push_back(false); //1-0 -> 1-1
    pathArray.push_back(false); //1-0 -> 2-0
    pathArray.push_back(false); //1-0 -> 2-1

    pathArray.push_back(false); //1-1 -> 0-0
    pathArray.push_back(false); //1-1 -> 0-1
    pathArray.push_back(false); //1-1 -> 1-0
    pathArray.push_back(false); //1-1 -> 1-1
    pathArray.push_back(false); //1-1 -> 2-0
    pathArray.push_back(false); //1-1 -> 2-1

    pathArray.push_back(false); //2-0 -> 0-0
    pathArray.push_back(false); //2-0 -> 0-1
    pathArray.push_back(false); //2-0 -> 1-0
    pathArray.push_back(false); //2-0 -> 1-1
    pathArray.push_back(false); //2-0 -> 2-0
    pathArray.push_back(false); //2-0 -> 2-1

    pathArray.push_back(false); //2-1 -> 0-0
    pathArray.push_back(false); //2-1 -> 0-1
    pathArray.push_back(false); //2-1 -> 1-0
    pathArray.push_back(false); //2-1 -> 1-1
    pathArray.push_back(false); //2-1 -> 2-0
    pathArray.push_back(false); //2-1 -> 2-1


    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "Invalid path with 2 edges", false);
}

BOOST_AUTO_TEST_CASE(multi_edge_path_parallel_2)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 3;
    size_t numberOfFluents = 2;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(false); //0-0 -> 0-1
    pathArray.push_back(true); //0-0 -> 1-0
    pathArray.push_back(false); //0-0 -> 1-1
    pathArray.push_back(false); //0-0 -> 2-0
    pathArray.push_back(false); //0-0 -> 2-1

    pathArray.push_back(false); //0-1 -> 0-0
    pathArray.push_back(false); //0-1 -> 0-1
    pathArray.push_back(false); //0-1 -> 1-0
    pathArray.push_back(false); //0-1 -> 1-1
    pathArray.push_back(false); //0-1 -> 2-0
    pathArray.push_back(false); //0-1 -> 2-1

    pathArray.push_back(false); //1-0 -> 0-0
    pathArray.push_back(false); //1-0 -> 0-1
    pathArray.push_back(false); //1-0 -> 1-0
    pathArray.push_back(false); //1-0 -> 1-1
    pathArray.push_back(true); //1-0 -> 2-0
    pathArray.push_back(false); //1-0 -> 2-1

    pathArray.push_back(false); //1-1 -> 0-0
    pathArray.push_back(false); //1-1 -> 0-1
    pathArray.push_back(false); //1-1 -> 1-0
    pathArray.push_back(false); //1-1 -> 1-1
    pathArray.push_back(true); //1-1 -> 2-0
    pathArray.push_back(false); //1-1 -> 2-1

    pathArray.push_back(false); //2-0 -> 0-0
    pathArray.push_back(false); //2-0 -> 0-1
    pathArray.push_back(false); //2-0 -> 1-0
    pathArray.push_back(false); //2-0 -> 1-1
    pathArray.push_back(false); //2-0 -> 2-0
    pathArray.push_back(false); //2-0 -> 2-1

    pathArray.push_back(false); //2-1 -> 0-0
    pathArray.push_back(false); //2-1 -> 0-1
    pathArray.push_back(false); //2-1 -> 1-0
    pathArray.push_back(false); //2-1 -> 1-1
    pathArray.push_back(false); //2-1 -> 2-0
    pathArray.push_back(false); //2-1 -> 2-1


    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "Invalid path with 2 edges", false);
}

BOOST_AUTO_TEST_CASE(multi_edge_path_3)
{
    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 3;
    size_t numberOfFluents = 2;
    pathArray.push_back(false); //0-0 -> 0-0
    pathArray.push_back(false); //0-0 -> 0-1
    pathArray.push_back(true); //0-0 -> 1-0
    pathArray.push_back(false); //0-0 -> 1-1
    pathArray.push_back(false); //0-0 -> 2-0
    pathArray.push_back(false); //0-0 -> 2-1

    pathArray.push_back(false); //0-1 -> 0-0
    pathArray.push_back(false); //0-1 -> 0-1
    pathArray.push_back(false); //0-1 -> 1-0
    pathArray.push_back(false); //0-1 -> 1-1
    pathArray.push_back(false); //0-1 -> 2-0
    pathArray.push_back(false); //0-1 -> 2-1

    pathArray.push_back(false); //1-0 -> 0-0
    pathArray.push_back(false); //1-0 -> 0-1
    pathArray.push_back(false); //1-0 -> 1-0
    pathArray.push_back(false); //1-0 -> 1-1
    pathArray.push_back(true); //1-0 -> 2-0
    pathArray.push_back(false); //1-0 -> 2-1

    pathArray.push_back(false); //1-1 -> 0-0
    pathArray.push_back(false); //1-1 -> 0-1
    pathArray.push_back(false); //1-1 -> 1-0
    pathArray.push_back(false); //1-1 -> 1-1
    pathArray.push_back(false); //1-1 -> 2-0
    pathArray.push_back(false); //1-1 -> 2-1

    pathArray.push_back(false); //2-0 -> 0-0
    pathArray.push_back(false); //2-0 -> 0-1
    pathArray.push_back(false); //2-0 -> 1-0
    pathArray.push_back(false); //2-0 -> 1-1
    pathArray.push_back(false); //2-0 -> 2-0
    pathArray.push_back(false); //2-0 -> 2-1

    pathArray.push_back(false); //2-1 -> 0-0
    pathArray.push_back(false); //2-1 -> 0-1
    pathArray.push_back(false); //2-1 -> 1-0
    pathArray.push_back(false); //2-1 -> 1-1
    pathArray.push_back(false); //2-1 -> 2-0
    pathArray.push_back(false); //2-1 -> 2-1

    testPathConstraint(pathArray, numberOfTimepoints, numberOfFluents, "Valid path with 2 edges", true);
}

BOOST_AUTO_TEST_SUITE_END()
