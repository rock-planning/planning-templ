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

    {
        Interval::List expected = { i0,i2,i3,i1 };
        FluentTimeResource::sortForEarlierEnd(requirements, tpc);

        Interval::List actual;
        for(const FluentTimeResource& ftr : requirements)
        {
            actual.push_back(ftr.getInterval());
        }

        BOOST_REQUIRE_MESSAGE(expected == actual, "Sorted list of intervals for"
                " earlier end should be as expected");
    }

    {
        Interval::List expected = { i0,i1,i2,i3 };
        FluentTimeResource::sortForEarlierStart(requirements, tpc);

        Interval::List actual;
        for(const FluentTimeResource& ftr : requirements)
        {
            actual.push_back(ftr.getInterval());
        }

        BOOST_REQUIRE_MESSAGE(expected == actual, "Sorted list of intervals for"
                " earlier start should be as expected");
    }
}

BOOST_AUTO_TEST_CASE(overlapping)
{
    std::string rootDir = getRootDir();
    std::string missionFilename = rootDir + "/test/data/scenarios/test-mission-ftrs-1.xml";

    Mission m = io::MissionReader::fromFile(missionFilename);
    Mission::Ptr mission(new Mission(m));
    mission->prepareTimeIntervals();

    FluentTimeResource::List requirements = Mission::getResourceRequirements(mission);
    mission->getTemporalConstraintNetwork()->isConsistent();

    TimePointComparator tpc(mission->getTemporalConstraintNetwork());
    TimePoint::Ptr t1 = TimePoint::create("t1");
    TimePoint::Ptr t2 = TimePoint::create("t2");
    TimePoint::Ptr t3 = TimePoint::create("t3");
    TimePoint::Ptr t4 = TimePoint::create("t4");
    TimePoint::Ptr t5 = TimePoint::create("t5");
    TimePoint::Ptr t6 = TimePoint::create("t6");
    TimePoint::Ptr t7 = TimePoint::create("t7");

    std::vector<FluentTimeResource::Set> overlap =
        FluentTimeResource::getOverlapping(requirements, tpc);

    for(FluentTimeResource::Set& o : overlap)
    {
        BOOST_TEST_MESSAGE("Overlapping requirements:" <<
                FluentTimeResource::toQualificationString(o.begin(),
                    o.end(),4));
    }
}

BOOST_AUTO_TEST_CASE(split_overlapping)
{
    std::string rootDir = getRootDir();
    std::string missionFilename = rootDir + "/test/data/scenarios/test-mission-ftrs-1.xml";

    Mission m = io::MissionReader::fromFile(missionFilename);
    Mission::Ptr mission(new Mission(m));
    mission->prepareTimeIntervals();

    FluentTimeResource::List requirements = Mission::getResourceRequirements(mission);
    mission->getTemporalConstraintNetwork()->isConsistent();

    TimePointComparator tpc(mission->getTemporalConstraintNetwork());
    TimePoint::Ptr t1 = TimePoint::create("t1");
    TimePoint::Ptr t2 = TimePoint::create("t2");
    TimePoint::Ptr t3 = TimePoint::create("t3");
    TimePoint::Ptr t4 = TimePoint::create("t4");
    TimePoint::Ptr t5 = TimePoint::create("t5");
    TimePoint::Ptr t6 = TimePoint::create("t6");
    TimePoint::Ptr t7 = TimePoint::create("t7");

    TimePoint::PtrList sortedTimepoints = { t1,t2,t3,t4,t5,t6,t7 };

    FluentTimeResource::List splitRequirements =
        Mission::getResourceRequirements(mission,
                sortedTimepoints,
                tpc);

    BOOST_TEST_MESSAGE("Split requirement: " <<
            FluentTimeResource::toString(splitRequirements) );
}

BOOST_AUTO_TEST_CASE(mutually_exclusive)
{
    std::string rootDir = getRootDir();
    std::string missionFilename = rootDir + "/test/data/scenarios/test-mission-ftrs-0.xml";

    Mission m = io::MissionReader::fromFile(missionFilename);
    Mission::Ptr mission = make_shared<Mission>(m);
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
    Interval i4(t6,t7, tpc);

    Interval::List a = { i0,i1};
    Interval::List b = { i0,i2};
    Interval::List c = { i1,i2 };
    Interval::List d = { i1,i3 };
    Interval::List e = { i1,i4 };
    Interval::List f = { i3,i4 };

    std::vector<Interval::List> expected = { a,b,c,d,e,f };

    std::vector<Interval::List> actual;
    std::vector<FluentTimeResource::List> mutuallyExclusiveSets = FluentTimeResource::getMutualExclusive(requirements, tpc);
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

    std::stringstream ssExcepted;
    for(const Interval::List& list : expected)
    {
        ssExcepted << "Exclusive" << std::endl;
        ssExcepted << Interval::toString(list,4);
    }

    std::stringstream ssActual;
    for(const Interval::List& list : actual)
    {
        ssActual << "Exclusive" << std::endl;
        ssActual << Interval::toString(list,4);
    }

    BOOST_REQUIRE_MESSAGE(expected == actual, "Concurrent intervals matches expected set of sets:"
            << "expected: " << ssExcepted.str() << "\n"
            << "actual: " << ssActual.str());
}

BOOST_AUTO_TEST_SUITE_END()
