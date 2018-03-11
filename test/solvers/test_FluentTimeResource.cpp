#include <boost/test/unit_test.hpp>
#include <templ/Mission.hpp>
#include <templ/io/MissionReader.hpp>
#include <templ/solvers/FluentTimeResource.hpp>

#include "../test_utils.hpp"

using namespace templ;
using namespace templ::solvers;
using namespace solvers::temporal;
using namespace solvers::temporal::point_algebra;

BOOST_AUTO_TEST_SUITE(fluent_time_resource)

BOOST_AUTO_TEST_CASE(sorting)
{
    std::string rootDir = getRootDir();
    std::string missionFilename = rootDir + "/test/data/scenarios/test-mission-ftrs-0.xml";

    Mission m = io::MissionReader::fromFile(missionFilename);
    Mission::Ptr mission(new Mission(m));
    mission->prepareTimeIntervals();

    FluentTimeResource::List requirements = Mission::getResourceRequirements(mission);
    mission->getTemporalConstraintNetwork()->isConsistent();

    TimePointComparator tpc(mission->getTemporalConstraintNetwork());
    mission->getTemporalConstraintNetwork()->save("/tmp/tcn");

    TimePoint::Ptr t1 = TimePoint::create("t1");
    TimePoint::Ptr t2 = TimePoint::create("t2");
    TimePoint::Ptr t3 = TimePoint::create("t3");
    TimePoint::Ptr t4 = TimePoint::create("t4");
    TimePoint::Ptr t5 = TimePoint::create("t5");
    TimePoint::Ptr t6 = TimePoint::create("t6");
    TimePoint::Ptr t7 = TimePoint::create("t7");

    BOOST_REQUIRE_MESSAGE( tpc.lessThan(t4,t6), "T4 should be less than t6");

    Interval i0(t1,t3, tpc);
    Interval i1(t1,t7, tpc);
    Interval i2(t2,t4, tpc);
    Interval i3(t5,t6, tpc);

    Interval::List expected = { i0,i2,i3,i1 };
    FluentTimeResource::sortForMutualExclusion(requirements);

    Interval::List actual;
    for(const FluentTimeResource& ftr : requirements)
    {
        actual.push_back(ftr.getInterval());
    }

    BOOST_REQUIRE_MESSAGE(expected == actual, "Sorted list of intervals should be as expected");
}

BOOST_AUTO_TEST_CASE(mutually_exclusive)
{
    std::string rootDir = getRootDir();
    std::string missionFilename = rootDir + "/test/data/scenarios/test-mission-ftrs-0.xml";

    Mission m = io::MissionReader::fromFile(missionFilename);
    Mission::Ptr mission(new Mission(m));
    mission->prepareTimeIntervals();

    FluentTimeResource::List requirements = Mission::getResourceRequirements(mission);
    mission->getTemporalConstraintNetwork()->isConsistent();

    TimePointComparator tpc(mission->getTemporalConstraintNetwork());
    mission->getTemporalConstraintNetwork()->save("/tmp/tcn");

    TimePoint::Ptr t1 = TimePoint::create("t1");
    TimePoint::Ptr t2 = TimePoint::create("t2");
    TimePoint::Ptr t3 = TimePoint::create("t3");
    TimePoint::Ptr t4 = TimePoint::create("t4");
    TimePoint::Ptr t5 = TimePoint::create("t5");
    TimePoint::Ptr t6 = TimePoint::create("t6");
    TimePoint::Ptr t7 = TimePoint::create("t7");

    BOOST_REQUIRE_MESSAGE( tpc.lessThan(t4,t6), "T4 should be less than t6");

    Interval i0(t1,t3, tpc);
    Interval i1(t1,t7, tpc);
    Interval i2(t2,t4, tpc);
    Interval i3(t5,t6, tpc);

    Interval::List a = { i0,i2,i1};
    Interval::List b = { i2,i1 };
    Interval::List c = { i3,i1 };

    std::vector<Interval::List> expected = { a,b,c};

    std::vector<Interval::List> actual;
    std::vector<FluentTimeResource::List> mutuallyExclusiveSets = FluentTimeResource::getMutualExclusive(requirements);
    for(const FluentTimeResource::List& concurrent : mutuallyExclusiveSets)
    {
        BOOST_TEST_MESSAGE("MutualExclusive: ");
        Interval::List concurrentIntervals;
        for(const FluentTimeResource& ftr : concurrent)
        {
            concurrentIntervals.push_back(ftr.getInterval());
            BOOST_TEST_MESSAGE("    " << ftr.getInterval().toString());
        }
        actual.push_back(concurrentIntervals);
    }
    BOOST_REQUIRE_MESSAGE(expected == actual, "Concurrent intervals matches expected set of sets");
}

BOOST_AUTO_TEST_SUITE_END()
