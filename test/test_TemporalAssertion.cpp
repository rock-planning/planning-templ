#include <boost/test/unit_test.hpp>
#include <templ/StateVariable.hpp>
#include <templ/Value.hpp>
#include <templ/values/Int.hpp>
#include <templ/solvers/temporal/TemporalAssertion.hpp>
#include <templ/solvers/temporal/Event.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>

using namespace templ;
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
    StateVariable stateVariable("robot_location", "robot_0");
    Value::Ptr fromValue0(new values::Int(0));
    Value::Ptr toValue0(new values::Int(10));
    Value::Ptr fromValue1(new values::Int(0));
    Value::Ptr toValue1(new values::Int(10));

    point_algebra::TimePoint::Ptr t0(new point_algebra::QualitativeTimePoint("t0"));
    point_algebra::TimePoint::Ptr t1(new point_algebra::QualitativeTimePoint("t1"));

    TemporalAssertion::Ptr event0(new Event(stateVariable, fromValue0, toValue0, t0));
    TemporalAssertion::Ptr event1(new Event(stateVariable, fromValue1, toValue1, t1));
}

BOOST_AUTO_TEST_SUITE_END()
