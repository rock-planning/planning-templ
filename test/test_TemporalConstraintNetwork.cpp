#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(temporal_constraint_network)

BOOST_AUTO_TEST_CASE(value_propagation)
{
	TemporalConstraintNetwork tcn1,tcn2;
/*
    point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,30));
    point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(20,40));

    tcn.addInterval(tp0, tp1, Bounds(5,10));
    tcn.addInterval(tp0, tp1, Bounds(3,8));

    //EdgeIterator::Ptr edgeIt = tcn->getEdgeIterator();
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 4,"Actual: "<<tcn.getEdgeNumber());

    point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(10,40));
    tcn.addInterval(tp2, tp0, Bounds(10,20));

    point_algebra::TimePoint::Ptr tp3(new point_algebra::TimePoint(20,30));
    tcn.addInterval(tp3, tp1, Bounds(1,2));
    tcn.addInterval(tp0, tp3, Bounds(2,3));
    tcn.addInterval(tp3, tp2, Bounds(6,7));
    tcn.addInterval(tp3, tp2, Bounds(8,9));

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 14,"Actual: "<<tcn.getEdgeNumber());

    int ok = tcn.stp();
    BOOST_REQUIRE_MESSAGE(ok == 4, "Actualx: " << ok);

    ok = tcn.stp();
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 10,"Actualy: "<<tcn.getEdgeNumber());
*/
    point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,20));
    point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(40,50));

    tcn1.addInterval(tp0, tp1, Bounds(20,60));
    tcn2.addInterval(tp0, tp1, Bounds(5,15));
    tcn2.addInterval(tp0, tp1, Bounds(40,80));

    BOOST_REQUIRE_MESSAGE(true == false,"Actual: "<<tcn1.intersection(tcn2));
}

BOOST_AUTO_TEST_SUITE_END()