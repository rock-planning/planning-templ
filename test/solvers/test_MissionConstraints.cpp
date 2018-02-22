#include <boost/test/unit_test.hpp>

#include <templ/solvers/csp/MissionConstraintManager.hpp>
#include <templ/Mission.hpp>
#include <templ/solvers/csp/TransportNetwork.hpp>
#include <templ/io/MissionReader.hpp>
#include <organization_model/vocabularies/OM.hpp>

#include "../test_utils.hpp"

using namespace templ;
using namespace templ::constraints;
using namespace templ::solvers::csp;


struct MissionFixture
{
    MissionFixture()
    {
    }

    ~MissionFixture()
    {}

    void setup(const std::string& filename = "test/data/scenarios/test-mission-constraints-base.xml")
    {
        baseMission = io::MissionReader::fromFile(getRootDir() +  filename);
        baseMission.prepareTimeIntervals();

        BOOST_TEST_MESSAGE("Mission: " << baseMission.toString());

        using namespace templ::solvers::csp;
        using namespace templ::symbols::constants;
        using namespace templ::solvers::temporal;

        Location::Ptr lander = baseMission.getLocation("lander");
        Location::Ptr base1  = baseMission.getLocation("base1");
        Location::Ptr base2  = baseMission.getLocation("base2");
        Location::Ptr base3  = baseMission.getLocation("base3");

        const Interval& i0   = baseMission.getTimeInterval("t0","t1");
        const Interval& i1   = baseMission.getTimeInterval("t2","t3");
        const Interval& i2   = baseMission.getTimeInterval("t4","t5");
        const Interval& i3   = baseMission.getTimeInterval("t6","t7");

        SpaceTime::SpaceIntervalTuple tuple0(lander, i0);
        SpaceTime::SpaceIntervalTuple tuple1(base1, i1);
        SpaceTime::SpaceIntervalTuple tuple2(base2, i2);
        SpaceTime::SpaceIntervalTuple tuple3(base3, i3);

        intervals.push_back(tuple1);
        intervals.push_back(tuple2);
        intervals.push_back(tuple3);
    }

    Mission baseMission;
    std::vector<SpaceTime::SpaceIntervalTuple> intervals;

    void solve(TransportNetwork* solver, Mission::Ptr& mission)
    {
        Gecode::BAB<TransportNetwork> searchEngine(solver);
        TransportNetwork* best = NULL;
        int i = 0;
        while(TransportNetwork* current = searchEngine.next())
        {
            if(i > 1)
            {
                break;
            }

            delete best;
            best = current;
            ++i;

            TransportNetwork::Solution solution = current->getSolution();
            current->saveSolution(solution, mission);
        }
    }

    void test(const templ::Constraint::Ptr& constraint = templ::Constraint::Ptr())
    {
        Mission::Ptr mission(new Mission(baseMission));
        TransportNetwork* solver = new TransportNetwork(mission);
        if(constraint)
        {
            solver->addConstraint(constraint);
        }
        solve(solver, mission);
        delete solver;
    }
};

BOOST_AUTO_TEST_SUITE(mission_constraints)

BOOST_FIXTURE_TEST_CASE(min, MissionFixture)
{
    setup();

    ModelConstraint::Ptr constraint = make_shared<ModelConstraint>(ModelConstraint::MIN,
            organization_model::vocabulary::OM::resolve("Payload"),
            intervals,
            2);

    test(constraint);

}

BOOST_FIXTURE_TEST_CASE(max, MissionFixture)
{
    setup();

    ModelConstraint::Ptr constraint = make_shared<ModelConstraint>(ModelConstraint::MAX,
            organization_model::vocabulary::OM::resolve("Payload"),
            intervals,
            1);

    test(constraint);

}

BOOST_FIXTURE_TEST_CASE(all_distinct, MissionFixture)
{
    setup();

    ModelConstraint::Ptr constraint = make_shared<ModelConstraint>(ModelConstraint::ALL_DISTINCT,
            organization_model::vocabulary::OM::resolve("Payload"),
            intervals);

    test(constraint);

}

BOOST_FIXTURE_TEST_CASE(min_distinct, MissionFixture)
{
    setup();

    ModelConstraint::Ptr constraint = make_shared<ModelConstraint>(ModelConstraint::MIN_DISTINCT,
            organization_model::vocabulary::OM::resolve("Payload"),
            intervals,
            2);

    test(constraint);
}

BOOST_FIXTURE_TEST_CASE(max_distinct, MissionFixture)
{
    setup();

    ModelConstraint::Ptr constraint = make_shared<ModelConstraint>(ModelConstraint::MAX_DISTINCT,
            organization_model::vocabulary::OM::resolve("Payload"),
            intervals,
            2);

    test(constraint);
}

BOOST_FIXTURE_TEST_CASE(all_equal, MissionFixture)
{
    setup();

    ModelConstraint::Ptr constraint = make_shared<ModelConstraint>(ModelConstraint::ALL_EQUAL,
            organization_model::vocabulary::OM::resolve("Payload"),
            intervals);

    test(constraint);
}

BOOST_FIXTURE_TEST_CASE(min_function, MissionFixture)
{
    setup();

    ModelConstraint::Ptr constraint = make_shared<ModelConstraint>(ModelConstraint::MIN_FUNCTION,
            organization_model::vocabulary::OM::resolve("TransportProvider"),
            intervals,
            1
            );

    test(constraint);
}

BOOST_FIXTURE_TEST_CASE(min_property, MissionFixture)
{
    setup();

    ModelConstraint::Ptr constraint = make_shared<ModelConstraint>(ModelConstraint::MIN_PROPERTY,
            organization_model::vocabulary::OM::resolve("TransportProvider"),
            intervals,
            3,
            organization_model::vocabulary::OM::resolve("transportCapacity")
            );

    test(constraint);
}

BOOST_FIXTURE_TEST_CASE(min_property_embedded, MissionFixture)
{
    setup("test/data/scenarios/test-mission-constraints-1.xml");
    test();
}

BOOST_AUTO_TEST_SUITE_END()
