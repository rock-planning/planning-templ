#include <boost/test/unit_test.hpp>
#include "../test_utils.hpp"

#include <templ/Mission.hpp>
#include <templ/io/MissionReader.hpp>
#include <templ/SpaceTime.hpp>
#include <templ/solvers/SolutionAnalysis.hpp>

using namespace templ;
using namespace templ::solvers;

struct SAFixture
{
    SpaceTime::Network solution;
    Mission::Ptr mission;

    SAFixture()
    {
        std::string filename = getRootDir() + "test/data/scenarios/test-mission-5.xml";
        std::string organizationModel = "http://www.rock-robotics.org/2015/12/projects#TransTerrA.owl";
        Mission m = io::MissionReader::fromFile(filename, organizationModel);
        mission = Mission::Ptr(new Mission(m));
    }
};

BOOST_FIXTURE_TEST_SUITE(solution_analysis, SAFixture)

BOOST_AUTO_TEST_CASE(required_roles)
{
    SolutionAnalysis sa(mission, solution);
    std::set<Role> roles = sa.getRequiredRoles();

}

BOOST_AUTO_TEST_CASE(available_resources)
{
    SolutionAnalysis sa(mission, solution);

    //std::vector<moreorg::ModelPool> getAvailableResources(location, interval);
}


BOOST_AUTO_TEST_SUITE_END()
