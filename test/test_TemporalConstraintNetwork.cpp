#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(temporal_constraint_network)

BOOST_AUTO_TEST_CASE(test_stp)
{
	TemporalConstraintNetwork tcn;

 //   point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,30));
 //   point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(20,40));

    Variable::Ptr v0(new Variable);
    Variable::Ptr v1(new Variable);
    Variable::Ptr v2(new Variable);

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1,5,10));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v0,v1,8,15));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v0,v1,4,20));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v0,v2,3,4));
    IntervalConstraint::Ptr i4(new IntervalConstraint(v1,v2,0,10));
    IntervalConstraint::Ptr i5(new IntervalConstraint(v1,v2,5,15));

//    tcn.addInterval(tp0, tp1, Bounds(5,10));
//    tcn.addInterval(tp0, tp1, Bounds(3,8));
    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    //EdgeIterator::Ptr edgeIt = tcn->getEdgeIterator();
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 2,"Actual: "<<tcn.getEdgeNumber());

//    point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(10,40));
//    tcn.addInterval(tp2, tp0, Bounds(10,20));
    tcn.addIntervalConstraint(i2);

//    point_algebra::TimePoint::Ptr tp3(new point_algebra::TimePoint(20,30));
    tcn.addIntervalConstraint(i3);
    tcn.addIntervalConstraint(i4);
    tcn.addIntervalConstraint(i5);

/*    tcn.addInterval(tp3, tp1, Bounds(1,2));
    tcn.addInterval(tp0, tp3, Bounds(2,3));
    tcn.addInterval(tp3, tp2, Bounds(6,7));
    tcn.addInterval(tp3, tp2, Bounds(8,9));
*/
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 6,"Expected: 6, Actual: "<<tcn.getEdgeNumber());

    tcn.stp();
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 3,"Expected: 3, Actual: "<<tcn.getEdgeNumber());
}

BOOST_AUTO_TEST_CASE(test_intersection)
{
    TemporalConstraintNetwork tcn0,tcn1;

    Variable::Ptr v0(new Variable);
    Variable::Ptr v1(new Variable);
    Variable::Ptr v2(new Variable);

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1,5,10));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v0,v1,8,15));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v0,v1,4,20));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v0,v2,3,4));
    IntervalConstraint::Ptr i4(new IntervalConstraint(v1,v2,0,10));
    IntervalConstraint::Ptr i5(new IntervalConstraint(v1,v2,5,15));
    
    tcn0.addIntervalConstraint(i0);
    tcn0.addIntervalConstraint(i1);
    tcn0.addIntervalConstraint(i2);
    tcn0.addIntervalConstraint(i3);
    tcn0.addIntervalConstraint(i4);
    tcn0.addIntervalConstraint(i5);

    IntervalConstraint::Ptr i6(new IntervalConstraint(v0,v1,11,14));
    IntervalConstraint::Ptr i7(new IntervalConstraint(v1,v2,3,13));
    IntervalConstraint::Ptr i8(new IntervalConstraint(v0,v2,1,3));
    
    tcn1.addIntervalConstraint(i6);
    tcn1.addIntervalConstraint(i7);
    tcn1.addIntervalConstraint(i8);

    BOOST_REQUIRE_MESSAGE(tcn0.getEdgeNumber() == 6,"Expected: 6, Actual: "<<tcn0.getEdgeNumber());
    BOOST_REQUIRE_MESSAGE(tcn1.getEdgeNumber() == 3,"Expected: 3, Actual: "<<tcn1.getEdgeNumber());

    tcn0.intersection(tcn1);

    BOOST_REQUIRE_MESSAGE(tcn0.getEdgeNumber() == 5,"Expected: 5, Actual: "<<tcn0.getEdgeNumber());
    BOOST_REQUIRE_MESSAGE(tcn1.getEdgeNumber() == 3,"Expected: 3, Actual: "<<tcn1.getEdgeNumber());

}

BOOST_AUTO_TEST_SUITE_END()