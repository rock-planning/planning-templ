#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/TemporalAssertion.hpp>
#include <templ/solvers/temporal/Event.hpp>
#include <templ/solvers/temporal/PersistenceCondition.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>
#include <templ/symbols/ObjectVariable.hpp>
#include <templ/symbols/StateVariable.hpp>
#include <templ/symbols/Value.hpp>
#include <templ/symbols/values/Int.hpp>

using namespace templ;
using namespace templ::symbols;
using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(temporal_assertion)

BOOST_AUTO_TEST_CASE(disjoint)
{
    StateVariable stateVariable("robot_location", "robot_0");
    Value::Ptr fromValue(new values::Int(0));
    Value::Ptr toValue(new values::Int(10));
    point_algebra::TimePoint::Ptr t0(new point_algebra::QualitativeTimePoint("t0"));
    point_algebra::TimePoint::Ptr t0_alias(new point_algebra::QualitativeTimePoint("t0"));
    point_algebra::TimePoint::Ptr t1(new point_algebra::QualitativeTimePoint("t1"));

    TemporalAssertion::Ptr event0a(new Event(stateVariable, fromValue, toValue, t0));
    TemporalAssertion::Ptr event0b(new Event(stateVariable, fromValue, toValue, t0));
    TemporalAssertion::Ptr event0a_alias(new Event(stateVariable, fromValue, toValue, t0_alias));
    TemporalAssertion::Ptr event1(new Event(stateVariable, fromValue, toValue, t1));

    QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());
    //qtcn->addConstraint(t0
    point_algebra::TimePointComparator comparator(qtcn);
    BOOST_REQUIRE_MESSAGE(!event0a->isDisjointFrom(event0b, comparator), "Identical events are not disjoint" );
    BOOST_REQUIRE_MESSAGE(!event0a->isDisjointFrom(event0a_alias, comparator), "Identical events are not disjoint" );
    BOOST_REQUIRE_MESSAGE(event0a->isDisjointFrom(event1, comparator), "Disjoint events are disjoint" );
}

BOOST_AUTO_TEST_CASE(same_value)
{
    QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());
    point_algebra::TimePointComparator comparator(qtcn);

    StateVariable stateVariable("robot_location", "robot_0");
    Value::Ptr fromValue0(new values::Int(0));
    Value::Ptr toValue0(new values::Int(10));
    Value::Ptr fromValue1(new values::Int(0));
    Value::Ptr toValue1(new values::Int(10));

    point_algebra::TimePoint::Ptr t0(new point_algebra::QualitativeTimePoint("t0"));
    point_algebra::TimePoint::Ptr t1(new point_algebra::QualitativeTimePoint("t1"));

    TemporalAssertion::Ptr event0(new Event(stateVariable, fromValue0, toValue0, t0));
    TemporalAssertion::Ptr event1(new Event(stateVariable, fromValue1, toValue1, t1));

    BOOST_REQUIRE_MESSAGE(event0->isReferringToSameValue(event1, comparator), "Two events refer to the same value");

    Value::Ptr fromValue2(new values::Int(0));
    Value::Ptr toValue2(new values::Int(2));
    TemporalAssertion::Ptr event2(new Event(stateVariable, fromValue2, toValue2, t0));
    BOOST_REQUIRE_MESSAGE(!event0->isReferringToSameValue(event2, comparator), "Two events refer to the same value");
    BOOST_REQUIRE_MESSAGE(!event1->isReferringToSameValue(event2, comparator), "Two events refer to the same value");

    {
        Value::Ptr valueFrom(new values::Int(10));
        Value::Ptr valueTo(new values::Int(30));

        point_algebra::TimePoint::Ptr timepoint0 = point_algebra::TimePoint::create("Initial timepoint");

        StateVariable stateVariable("location", "sherpa");

        point_algebra::TimePointComparator comparator;

        Event::Ptr event0(new Event(stateVariable, valueFrom, valueTo, timepoint0));
        Event::Ptr event1(new Event(stateVariable, valueFrom, valueTo, timepoint0));

        BOOST_REQUIRE_MESSAGE(event0->isReferringToSameValue(event1, comparator), "Same event refers to same value");

        Value::Ptr valueFrom2(new values::Int(20));
        Value::Ptr valueTo2(new values::Int(40));

        Event::Ptr event2(new Event(stateVariable, valueFrom2, valueTo2, timepoint0));
        BOOST_REQUIRE_MESSAGE(!event0->isReferringToSameValue(event2, comparator), "Different events refer not to the same value");
    }
}

BOOST_AUTO_TEST_CASE(persistence_condition)
{
    StateVariable stateVariable("robot_location","robot0");
    {
        // rloc(robot0)@[t2,t3) : l1
        ObjectVariable::Ptr objectVariable0(new ObjectVariable("l1"));
        point_algebra::TimePoint::Ptr fromT(new point_algebra::QualitativeTimePoint("t2"));
        point_algebra::TimePoint::Ptr toT(new point_algebra::QualitativeTimePoint("t3"));

        PersistenceCondition::Ptr assertion0(new PersistenceCondition(stateVariable, objectVariable0, fromT, toT));

        // rloc(robot0)@[t2,t3) : l3
        ObjectVariable::Ptr objectVariable1(new ObjectVariable("l3"));
        PersistenceCondition::Ptr assertion1(new PersistenceCondition(stateVariable, objectVariable1, fromT, toT));

        QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());
        qtcn->addConstraint(fromT, toT, point_algebra::QualitativeTimePointConstraint::LessOrEqual);

        qtcn->addTimePoint(fromT);
        qtcn->addTimePoint(toT);

        point_algebra::TimePointComparator comparator(qtcn);
        BOOST_REQUIRE_MESSAGE(!assertion0->isDisjointFrom(assertion1, comparator), "Overlapping persistence conditions are not disjoint");

        // Check if there is no connecting constraint between two intervals
        point_algebra::TimePoint::Ptr fromT2(new point_algebra::QualitativeTimePoint("t4"));
        point_algebra::TimePoint::Ptr toT2(new point_algebra::QualitativeTimePoint("t5"));
        qtcn->addConstraint(fromT2, toT2, point_algebra::QualitativeTimePointConstraint::LessOrEqual);

        PersistenceCondition::Ptr assertion2(new PersistenceCondition(stateVariable, objectVariable1, fromT2, toT2));
        BOOST_REQUIRE_MESSAGE(assertion0->isDisjointFrom(assertion2, comparator), "Non overlapping persistence conditions are disjoint");

        qtcn->addConstraint(fromT2, fromT, point_algebra::QualitativeTimePointConstraint::Equal);
        BOOST_REQUIRE_MESSAGE(!assertion0->isDisjointFrom(assertion2, comparator), "Overlapping persistence conditions are not disjoint");
    }

}

BOOST_AUTO_TEST_SUITE_END()
