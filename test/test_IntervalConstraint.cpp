#include <boost/test/unit_test.hpp>
#include "test_utils.hpp"

#include <templ/solvers/temporal/IntervalConstraint.hpp>

using namespace templ::solvers::temporal;
using namespace templ::solvers::temporal::point_algebra;

BOOST_AUTO_TEST_SUITE(interval_constraint)

BOOST_AUTO_TEST_CASE(serialization)
{
    {
        TimePoint::Ptr tp0(new TimePoint(10,10));
        TimePoint::Ptr tp1(new TimePoint(11,11));

        IntervalConstraint i(tp0, tp1);
        i.addInterval(Bounds(0,10));

        std::string data = i.serializeBounds();
        BOOST_TEST_MESSAGE("test i: " << data);

        IntervalConstraint iDup;
        iDup.deserializeBounds(data);
        std::string dupData = iDup.serializeBounds();
        BOOST_REQUIRE_MESSAGE(data == dupData, "Expected " << data << " to equal clone " << dupData);

    }
}


BOOST_AUTO_TEST_SUITE_END()
