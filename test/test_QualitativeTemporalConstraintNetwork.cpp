#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(qualitative_temporal_constraint_network)

BOOST_AUTO_TEST_CASE(timepoint_comparision)
{
    {
        QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());

        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));

        qtcn->addConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::Greater);

        std::vector<point_algebra::QualitativeTimePointConstraint::Ptr> constraints = qtcn->getConstraints(tp0, tp1);
        BOOST_REQUIRE_MESSAGE(constraints.size() == 1, "Query constraint between two vertices");

        qtcn->addConstraint(tp1, tp2, point_algebra::QualitativeTimePointConstraint::Greater);

        point_algebra::TimePointComparator comparator(qtcn);
        BOOST_REQUIRE_MESSAGE(comparator.greaterThan(tp0, tp1), "tp0 greaterThan tp1");
        BOOST_REQUIRE_MESSAGE(!comparator.equals(tp0, tp1), "tp0 not equals tp1");

        BOOST_REQUIRE_MESSAGE(comparator.greaterThan(tp1, tp2), "tp1 greaterThan tp2");
    }
    {
        QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());

        point_algebra::TimePointComparator comparator(qtcn);

        point_algebra::TimePoint::Ptr t0_start(new point_algebra::QualitativeTimePoint("t0_start"));
        point_algebra::TimePoint::Ptr t0_end(new point_algebra::QualitativeTimePoint("t0_end"));
        point_algebra::TimePoint::Ptr t1_start(new point_algebra::QualitativeTimePoint("t1_start"));
        point_algebra::TimePoint::Ptr t1_end(new point_algebra::QualitativeTimePoint("t1_end"));

        qtcn->addConstraint(t0_start, t0_end, point_algebra::QualitativeTimePointConstraint::LessOrEqual);
        qtcn->addConstraint(t1_start, t1_end, point_algebra::QualitativeTimePointConstraint::LessOrEqual);

        BOOST_REQUIRE_MESSAGE(!comparator.hasIntervalOverlap(t0_start, t0_end, t1_start, t1_end), "No interval overlap when not constrained is defined");

        qtcn->addConstraint(t0_start, t1_end, point_algebra::QualitativeTimePointConstraint::Less);
        qtcn->addConstraint(t0_end, t1_start, point_algebra::QualitativeTimePointConstraint::Greater);

        BOOST_REQUIRE_MESSAGE(comparator.hasIntervalOverlap(t0_start, t0_end, t1_start, t1_end), "Interval overlap");
    }

}

BOOST_AUTO_TEST_CASE(consistency_checking)
{
    {
        QualitativeTemporalConstraintNetwork qtcn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));


        qtcn.addConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
        qtcn.addConstraint(tp1, tp2, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);

        BOOST_REQUIRE_MESSAGE(qtcn.isConsistent(), "qtcn is consistent");
    }
    {
        QualitativeTemporalConstraintNetwork qtcn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));

        qtcn.addConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
        qtcn.addConstraint(tp1, tp2, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
        qtcn.addConstraint(tp0, tp2, point_algebra::QualitativeTimePointConstraint::Less);

        BOOST_REQUIRE_MESSAGE(!qtcn.isConsistent(), "qtcn is inconsistent");
    }
    {
        QualitativeTemporalConstraintNetwork qtcn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));

        qtcn.addConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::Greater);
        qtcn.addConstraint(tp1, tp2, point_algebra::QualitativeTimePointConstraint::Less);
        qtcn.addConstraint(tp0,tp2,  point_algebra::QualitativeTimePointConstraint::Less);

        BOOST_REQUIRE_MESSAGE(qtcn.isConsistent(), "qtcn is consistent");
    }

    {
        QualitativeTemporalConstraintNetwork qtcn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));

        qtcn.addConstraint(tp0,tp1, point_algebra::QualitativeTimePointConstraint::Less);
        qtcn.addConstraint(tp1,tp0, point_algebra::QualitativeTimePointConstraint::Greater);

        BOOST_REQUIRE_MESSAGE(qtcn.isConsistent(), "qtcn is consistent for exact timepoint");
    }
    {
        QualitativeTemporalConstraintNetwork qtcn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));

        qtcn.addConstraint(tp0,tp1, point_algebra::QualitativeTimePointConstraint::Greater);
        qtcn.addConstraint(tp1,tp0, point_algebra::QualitativeTimePointConstraint::Greater);

        BOOST_REQUIRE_MESSAGE(!qtcn.isConsistent(), "qtcn is not consistent for contradicting timepoint relationships");
    }
    {
        QualitativeTemporalConstraintNetwork qtcn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));

        qtcn.addConstraint(tp0,tp1, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
        qtcn.addConstraint(tp0,tp1, point_algebra::QualitativeTimePointConstraint::Less);

        BOOST_REQUIRE_MESSAGE(!qtcn.isConsistent(), "qtcn is not consistent for contradicting timepoint relationships");
    }
}


BOOST_AUTO_TEST_SUITE_END()
