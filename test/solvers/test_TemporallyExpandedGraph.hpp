#include <boost/test/unit_test.hpp>
#include "transport_network/TestSpace.hpp"
#include <gecode/search.hh>

using namespace templ;
BOOST_AUTO_TEST_SUITE(csp_temporally_expanded_graph)

BOOST_AUTO_TEST_CASE(is_path)
{
    using namespace templ::test;

    std::vector<bool> pathArray;
    size_t numberOfTimepoints = 1;
    size_t numberOfFluents = 1;

    pathArray.push_back(true);

    // construct a path based on the activation
    // array: timepointIdx*#fluents + fluentIdx


    TestSpace* testSpace = new TestSpace(pathArray, numberOfTimepoints, numberOfFluents);
    Gecode::BAB<TestSpace> searchEngine(testSpace);

    TestSpace* first = searchEngine.next();
    BOOST_REQUIRE_MESSAGE(first != NULL, "Found a solution to the path problem");
    delete first;
}

BOOST_AUTO_TEST_SUITE_END()
