#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>
#include <templ/Constraint.hpp>

using namespace templ;
using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(qualitative_temporal_constraint_network)

BOOST_AUTO_TEST_CASE(constraint)
{
    point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
    point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
    point_algebra::QualitativeTimePointConstraint qtcn(tp0, tp1, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
    templ::Constraint::Category category = qtcn.getCategory();
    BOOST_REQUIRE_MESSAGE( category == templ::Constraint::TEMPORAL_QUALITATIVE, "Qual: QualitativeType expected, but got: " << templ::Constraint::CategoryTxt[category]);
}

BOOST_AUTO_TEST_CASE(timepoint_comparison)
{
    {
        QualitativeTemporalConstraintNetwork::Ptr qtcn(new QualitativeTemporalConstraintNetwork());

        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));

        qtcn->addQualitativeConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::Greater);
        BOOST_REQUIRE_MESSAGE( qtcn->isConsistent(), "QualitativeTemporalConstraintNetwork is consistent" );

        point_algebra::QualitativeTimePointConstraint::Type constraint = qtcn->getQualitativeConstraint(tp0, tp1);
        BOOST_REQUIRE_MESSAGE(constraint == point_algebra::QualitativeTimePointConstraint::Greater, "Query result of constraint between two vertices" << point_algebra::QualitativeTimePointConstraint::TypeTxt[constraint]);

        qtcn->addQualitativeConstraint(tp1, tp2, point_algebra::QualitativeTimePointConstraint::Greater);

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

        // t0_s <= t0_e
        // t1_s <= t1_e
        //
        qtcn->addQualitativeConstraint(t0_start, t0_end, point_algebra::QualitativeTimePointConstraint::LessOrEqual);
        qtcn->addQualitativeConstraint(t1_start, t1_end, point_algebra::QualitativeTimePointConstraint::LessOrEqual);
        BOOST_REQUIRE_MESSAGE(qtcn->isConsistent(), "Network is consistent");

        BOOST_REQUIRE_MESSAGE(!comparator.hasIntervalOverlap(t0_start, t0_end, t1_start, t1_end), "No interval overlap when not constrained is defined");

        // t0_s <= t1_e
        // t0_e > t1_s
        //
        // [t0_s     t0_e]
        //      [t1_s       t1_e]
        qtcn->addQualitativeConstraint(t0_start, t1_end, point_algebra::QualitativeTimePointConstraint::Less);
        qtcn->addQualitativeConstraint(t0_end, t1_start, point_algebra::QualitativeTimePointConstraint::Greater);
        BOOST_REQUIRE_MESSAGE(qtcn->isConsistent(), "Network is consistent after adding overlap constraints");

        {
            point_algebra::QualitativeTimePointConstraint::Type constraint0 = qtcn->getQualitativeConstraint(t0_start, t1_end);
            point_algebra::QualitativeTimePointConstraint::Type constraint1 = qtcn->getQualitativeConstraint(t1_end, t0_start);
            BOOST_REQUIRE_MESSAGE(constraint0 == point_algebra::QualitativeTimePointConstraint::Less || constraint1 == point_algebra::QualitativeTimePointConstraint::GreaterOrEqual, "Query result of constraint between two vertices " << point_algebra::QualitativeTimePointConstraint::TypeTxt[constraint0] <<  "--"<< point_algebra::QualitativeTimePointConstraint::TypeTxt[constraint1]);
            BOOST_REQUIRE_MESSAGE(comparator.lessThan(t0_start, t1_end), "t0_start < t1_end");
        }
        {
            point_algebra::QualitativeTimePointConstraint::Type constraint0 = qtcn->getQualitativeConstraint(t0_end, t1_start);
            point_algebra::QualitativeTimePointConstraint::Type constraint1 = qtcn->getQualitativeConstraint(t1_start, t0_end);
            BOOST_REQUIRE_MESSAGE(constraint0 == point_algebra::QualitativeTimePointConstraint::Greater || constraint1 == point_algebra::QualitativeTimePointConstraint::LessOrEqual, "Query result of constraint between two vertices " << point_algebra::QualitativeTimePointConstraint::TypeTxt[constraint0] << "--" << point_algebra::QualitativeTimePointConstraint::TypeTxt[constraint1]);
            BOOST_REQUIRE_MESSAGE(comparator.greaterThan(t0_end, t1_start), "t0_end > t1_start");
        }

        // check t0_end and t1_start: hasOverlap if !(t0_end <= t1_start || t1_end <= t0_start )
        //
        // t0_start < t1_end
        // t0_end > t1_start
        BOOST_REQUIRE_MESSAGE(!comparator.lessOrEqual(t0_end, t1_start), "NOT(!) t0_end <= t1_start");
        BOOST_REQUIRE_MESSAGE(comparator.hasIntervalOverlap(t0_start, t0_end, t1_start, t1_end), "Interval overlap");

        qtcn->addQualitativeConstraint(t0_end, t1_end, point_algebra::QualitativeTimePointConstraint::Less);
        BOOST_REQUIRE_MESSAGE(comparator.hasIntervalOverlap(t0_start, t0_end, t0_end, t1_end), "Interval overlap with identical end and startpoint expected");
    }

}

BOOST_AUTO_TEST_CASE(consistency_checking)
{
//    {
//        QualitativeTemporalConstraintNetwork qtcn;
//        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
//        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
//        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));
//
//
//        qtcn.addQualitativeConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
//        qtcn.addQualitativeConstraint(tp1, tp2, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
//
//        BOOST_REQUIRE_MESSAGE(qtcn.isConsistent(), "qtcn is consistent");
//    }
//    {
//        QualitativeTemporalConstraintNetwork qtcn;
//        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
//        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
//        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));
//
//        qtcn.addQualitativeConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
//        qtcn.addQualitativeConstraint(tp1, tp2, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
//        qtcn.addQualitativeConstraint(tp0, tp2, point_algebra::QualitativeTimePointConstraint::Less);
//
//        BOOST_REQUIRE_MESSAGE(!qtcn.isConsistent(), "qtcn is inconsistent: tp0 should be >= t2");
//    }
//    {
//        QualitativeTemporalConstraintNetwork qtcn;
//        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
//        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
//        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));
//
//        qtcn.addQualitativeConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::Greater);
//        qtcn.addQualitativeConstraint(tp1, tp2, point_algebra::QualitativeTimePointConstraint::Less);
//        qtcn.addQualitativeConstraint(tp0,tp2,  point_algebra::QualitativeTimePointConstraint::Less);
//
//        BOOST_REQUIRE_MESSAGE(qtcn.isConsistent(), "qtcn is consistent");
//    }
//
//    {
//        QualitativeTemporalConstraintNetwork qtcn;
//        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
//        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
//
//        qtcn.addQualitativeConstraint(tp0,tp1, point_algebra::QualitativeTimePointConstraint::Less);
//        qtcn.addQualitativeConstraint(tp1,tp0, point_algebra::QualitativeTimePointConstraint::Greater);
//
//        BOOST_REQUIRE_MESSAGE(qtcn.isConsistent(), "qtcn is consistent for exact timepoint");
//    }
//    {
//        QualitativeTemporalConstraintNetwork qtcn;
//        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
//        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
//
//        qtcn.addQualitativeConstraint(tp0,tp1, point_algebra::QualitativeTimePointConstraint::Greater);
//        qtcn.addQualitativeConstraint(tp1,tp0, point_algebra::QualitativeTimePointConstraint::Greater);
//
//        BOOST_REQUIRE_MESSAGE(!qtcn.isConsistent(), "qtcn should be not consistent for contradicting timepoint relationships");
//    }
    {
        QualitativeTemporalConstraintNetwork qtcn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));

        qtcn.addQualitativeConstraint(tp0,tp1, point_algebra::QualitativeTimePointConstraint::GreaterOrEqual);
        qtcn.addQualitativeConstraint(tp0,tp1, point_algebra::QualitativeTimePointConstraint::Less);

        BOOST_REQUIRE_MESSAGE(!qtcn.isConsistent(), "qtcn is not consistent for contradicting timepoint relationships");
    }
}


BOOST_AUTO_TEST_SUITE_END()
