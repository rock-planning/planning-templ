#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(temporal_constraint_network)

BOOST_AUTO_TEST_CASE(value_propagation)
{
	TemporalConstraintNetwork tcn,tcn2;

    point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,30));
    point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(20,40));

    Variable::Ptr v0(new Variable);
    Variable::Ptr v1(new Variable);
    Variable::Ptr v2(new Variable);
    Variable::Ptr v3(new Variable);

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1,5,10));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v0,v1,3,8));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v2,v0,10,20));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v3,v1,1,2));
    IntervalConstraint::Ptr i4(new IntervalConstraint(v0,v3,2,3));
    IntervalConstraint::Ptr i5(new IntervalConstraint(v3,v2,6,7));
    IntervalConstraint::Ptr i6(new IntervalConstraint(v3,v2,8,9));

//    tcn.addInterval(tp0, tp1, Bounds(5,10));
//    tcn.addInterval(tp0, tp1, Bounds(3,8));
    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    //EdgeIterator::Ptr edgeIt = tcn->getEdgeIterator();
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 2,"Actual: "<<tcn.getEdgeNumber());

    point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(10,40));
//    tcn.addInterval(tp2, tp0, Bounds(10,20));
    tcn.addIntervalConstraint(i2);

    point_algebra::TimePoint::Ptr tp3(new point_algebra::TimePoint(20,30));
    tcn.addIntervalConstraint(i3);
    tcn.addIntervalConstraint(i4);
    tcn.addIntervalConstraint(i5);
    tcn.addIntervalConstraint(i6);

/*    tcn.addInterval(tp3, tp1, Bounds(1,2));
    tcn.addInterval(tp0, tp3, Bounds(2,3));
    tcn.addInterval(tp3, tp2, Bounds(6,7));
    tcn.addInterval(tp3, tp2, Bounds(8,9));
*/
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 7,"Actual: "<<tcn.getEdgeNumber());

    int ok = tcn.stp();
    BOOST_REQUIRE_MESSAGE(ok == 4, "Actualx: " << ok);

    ok = tcn.stp();
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 10,"Actualy: "<<tcn.getEdgeNumber());
/*
    point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,20));
    point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(40,50));

    Variable::Ptr v1(new Variable);
    Variable::Ptr v2(new Variable);
    IntervalConstraint::Ptr a(new IntervalConstraint(v1,v2,20,60));
    IntervalConstraint::Ptr b(new IntervalConstraint(v1,v2,5,15));
    IntervalConstraint::Ptr c(new IntervalConstraint(v1,v2,40,80));
    
    tcn1.addIntervalConstraint(a);
    tcn2.addIntervalConstraint(b);
    tcn2.addIntervalConstraint(c);
*/
    //BOOST_REQUIRE_MESSAGE(true == false,"Actual: "<<tcn1.intersection(tcn2));
}

BOOST_AUTO_TEST_SUITE_END()