#include <boost/test/unit_test.hpp>
#include <templ/symbols/Value.hpp>
#include <templ/symbols/values/Int.hpp>
#include <templ/solvers/temporal/Event.hpp>
#include <templ/solvers/temporal/PersistenceCondition.hpp>
#include <templ/solvers/temporal/Timeline.hpp>

using namespace templ;
using namespace templ::symbols;
using namespace templ::solvers::temporal;

namespace pa = templ::solvers::temporal::point_algebra;

BOOST_AUTO_TEST_SUITE(timeline)

BOOST_AUTO_TEST_CASE(persistence_conditions)
{
    StateVariable stateVariable("robot_location","robot0");
    Timeline timeline(stateVariable);

    // rloc(robot0)@[t0,t1) : l0
    ObjectVariable::Ptr l0(new ObjectVariable("l0"));
    ObjectVariable::Ptr l1(new ObjectVariable("l1"));
    ObjectVariable::Ptr l2(new ObjectVariable("l2"));

    point_algebra::TimePoint::Ptr t0(new point_algebra::QualitativeTimePoint("t0"));
    point_algebra::TimePoint::Ptr t1(new point_algebra::QualitativeTimePoint("t1"));
    point_algebra::TimePoint::Ptr t2(new point_algebra::QualitativeTimePoint("t2"));
    point_algebra::TimePoint::Ptr t3(new point_algebra::QualitativeTimePoint("t3"));


    PersistenceCondition::Ptr assertion0(new PersistenceCondition(stateVariable, l0, t0, t1));
    timeline.addTemporalAssertion(assertion0);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with 1 assertion is consistent");

    PersistenceCondition::Ptr assertion1(new PersistenceCondition(stateVariable, l0, t0, t1));
    timeline.addTemporalAssertion(assertion1);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with same assertions consistent");

    {
        // rloc(robot0)@[t2,t3) : l1
        PersistenceCondition::Ptr assertion1(new PersistenceCondition(stateVariable, l1, t2, t3));
        timeline.addTemporalAssertion(assertion1);
        BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with 1 assertion is consistent");
    }

    {
        // rloc(robot0)@[t2,t3) : l3
        PersistenceCondition::Ptr assertion1(new PersistenceCondition(stateVariable, l2, t2, t3));
        timeline.addTemporalAssertion(assertion1);
        BOOST_REQUIRE_MESSAGE(!timeline.isConsistent(), "Timeline with 1 assertion is inconsistent");
    }
}

BOOST_AUTO_TEST_CASE(events)
{
    StateVariable stateVariable("robot_location","robot0");
    Timeline timeline(stateVariable);

    // rloc(robot0)@[t0,t1) : l0
    ObjectVariable::Ptr l0 = ObjectVariable::getInstance("l0");
    ObjectVariable::Ptr l1 = ObjectVariable::getInstance("l1");
    ObjectVariable::Ptr l2 = ObjectVariable::getInstance("l2");

    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");

    Event::Ptr assertion0 = Event::getInstance(stateVariable, l0, l1, t0);
    Event::Ptr assertion1 = Event::getInstance(stateVariable, l1, l2, t1);
    Event::Ptr assertion2 = Event::getInstance(stateVariable, l2, l2, t2);
    Event::Ptr assertion3 = Event::getInstance(stateVariable, l1, l2, t2);

    timeline.addTemporalAssertion(assertion0);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with 1 assertion is consistent");
    timeline.addTemporalAssertion(assertion0);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with same assertions consistent");
    timeline.addTemporalAssertion(assertion1);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with 3 valid assertions consistent");
    timeline.addTemporalAssertion(assertion2);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with 4 valid assertions consistent");

    timeline.addTemporalAssertion(assertion3);
    BOOST_REQUIRE_MESSAGE(!timeline.isConsistent(), "Timeline with invalid assertions is not consistent");
}

BOOST_AUTO_TEST_CASE(persistence_conditions_and_events)
{
    StateVariable stateVariable("robot_location","robot0");
    Timeline timeline(stateVariable);

    // rloc(robot0)@[t0,t1) : l0
    ObjectVariable::Ptr l0 = ObjectVariable::getInstance("l0");
    ObjectVariable::Ptr l1 = ObjectVariable::getInstance("l1");
    ObjectVariable::Ptr l2 = ObjectVariable::getInstance("l2");

    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");

    Event::Ptr event0 = Event::getInstance(stateVariable, l0, l1, t0);
    Event::Ptr event1 = Event::getInstance(stateVariable, l1, l2, t1);
    Event::Ptr event2 = Event::getInstance(stateVariable, l2, l2, t2);
    Event::Ptr event3 = Event::getInstance(stateVariable, l1, l2, t2);

    PersistenceCondition::Ptr pc0 = PersistenceCondition::getInstance(stateVariable, l1, t0, t1);
    PersistenceCondition::Ptr pc1 = PersistenceCondition::getInstance(stateVariable, l2, t1, t2);

    timeline.addTemporalAssertion(event0);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with one event consistent");
    timeline.addTemporalAssertion(pc0);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with one event and one persistence condition consistent");
    timeline.addTemporalAssertion(event1);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with two events and one persistence condition consistent");
    timeline.addTemporalAssertion(pc1);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with two events and two persistence conditions consistent");
    timeline.addTemporalAssertion(event2);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with three events and two persistence conditions is consistent");
    timeline.addTemporalAssertion(event3);
    BOOST_REQUIRE_MESSAGE(!timeline.isConsistent(), "Timeline with four events and two persistence conditions is inconsistent");
}

BOOST_AUTO_TEST_SUITE_END()
