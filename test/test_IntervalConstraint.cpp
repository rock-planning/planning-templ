#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/IntervalConstraint.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(interval_constraint)

BOOST_AUTO_TEST_CASE(test)
{
	Variable::Ptr v1(new Variable);
	Variable::Ptr v2(new Variable);
	IntervalConstraint::Ptr a(new IntervalConstraint(v1,v2,5,10));

	//BOOST_ERROR(a.toString());
	//BOOST_REQUIRE_MESSAGE(true == false,a.toString());

	TemporalConstraintNetwork tcn;
	tcn.addIntervalConstraint(a);
}


BOOST_AUTO_TEST_SUITE_END()
