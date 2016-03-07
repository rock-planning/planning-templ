#include <boost/test/unit_test.hpp>
#include "test_utils.hpp"

#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>

using namespace templ::solvers::temporal::point_algebra;

BOOST_AUTO_TEST_SUITE(timepoint)

BOOST_AUTO_TEST_CASE(equality)
{
    {
        TimePoint::Ptr tp0(new TimePoint(0,10));
        TimePoint::Ptr tp0Dup(new TimePoint(0,10));
        TimePoint::Ptr tp1(new TimePoint(0,11));

        BOOST_REQUIRE_MESSAGE(tp0->equals(tp0Dup), tp0->toString() << " equals " << tp0Dup->toString());
        BOOST_REQUIRE_MESSAGE(!tp0->equals(tp1), tp0->toString() << " does not equal " << tp1->toString());
    }

    {
        TimePoint::Ptr tp0 = TimePoint::create("tp0");
        TimePoint::Ptr tp0Dup = TimePoint::create("tp0");
        TimePoint::Ptr tp1 = TimePoint::create("tp1");

        BOOST_REQUIRE_MESSAGE(tp0->equals(tp0Dup), tp0->toString() << " equals " << tp0Dup->toString());
        BOOST_REQUIRE_MESSAGE(!tp0->equals(tp1), tp0->toString() << " does not equal " << tp1->toString());

        QualitativeTimePoint::Ptr qtp = templ::dynamic_pointer_cast<QualitativeTimePoint>(tp1);
        qtp->addAlias("tp0");

        BOOST_REQUIRE_MESSAGE(tp0->equals(tp1), tp0->toString() << " equals (through alias) " << tp1->toString());
    }
}


BOOST_AUTO_TEST_SUITE_END()
