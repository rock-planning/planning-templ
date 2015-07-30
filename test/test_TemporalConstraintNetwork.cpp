#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(temporal_constraint_network)

BOOST_AUTO_TEST_CASE(value_propagation)
{
	TemporalConstraintNetwork tcn;

    point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,30));
    point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(20,40));

    tcn.addInterval(tp0, tp1, Bounds(5,10));
    tcn.addInterval(tp0, tp1, Bounds(3,8));

    //EdgeIterator::Ptr edgeIt = tcn->getEdgeIterator();
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 4,"Actual: "<<tcn.getEdgeNumber());

    point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(10,40));
    tcn.addInterval(tp2, tp0, Bounds(10,20));

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 6,"Actual: "<<tcn.getEdgeNumber());

    BOOST_REQUIRE_MESSAGE(tcn.stp() == 2, "Actualx: " << tcn.stp());

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 4,"Actualy: "<<tcn.getEdgeNumber());
}

BOOST_AUTO_TEST_SUITE_END()