#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/SimpleTemporalNetwork.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(simple_temporal_network)

BOOST_AUTO_TEST_CASE(consistency_checking)
{
    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));


        stn.addQualitativeConstraint(tp0, tp1, point_algebra::GreaterOrEqual);
        stn.addQualitativeConstraint(tp1, tp2, point_algebra::GreaterOrEqual);

        BOOST_REQUIRE_MESSAGE(stn.isConsistent(), "STN is consistent");
    }
    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));

        stn.addQualitativeConstraint(tp0, tp1, point_algebra::GreaterOrEqual);
        stn.addQualitativeConstraint(tp1, tp2, point_algebra::GreaterOrEqual);
        stn.addQualitativeConstraint(tp0, tp2, point_algebra::Less);

        BOOST_REQUIRE_MESSAGE(!stn.isConsistent(), "STN is inconsistent");
    }
    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::QualitativeTimePoint("tp2"));

        stn.addQualitativeConstraint(tp0, tp1, point_algebra::Greater);
        stn.addQualitativeConstraint(tp1, tp2, point_algebra::Less);
        stn.addQualitativeConstraint(tp0,tp2, point_algebra::Less);

        BOOST_REQUIRE_MESSAGE(stn.isConsistent(), "STN is consistent");
    }

    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));

        stn.addQualitativeConstraint(tp0,tp1, point_algebra::Less);
        stn.addQualitativeConstraint(tp1,tp0, point_algebra::Greater);

        BOOST_REQUIRE_MESSAGE(stn.isConsistent(), "STN is consistent for exact timepoint");
    }
    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));

        stn.addQualitativeConstraint(tp0,tp1, point_algebra::Greater);
        stn.addQualitativeConstraint(tp1,tp0, point_algebra::Greater);

        BOOST_REQUIRE_MESSAGE(!stn.isConsistent(), "STN is not consistent for contradicting timepoint relationships");
    }
    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));

        stn.addQualitativeConstraint(tp0,tp1, point_algebra::GreaterOrEqual);
        stn.addQualitativeConstraint(tp0,tp1, point_algebra::Less);

        BOOST_REQUIRE_MESSAGE(!stn.isConsistent(), "STN is not consistent for contradicting timepoint relationships");
    }
}

BOOST_AUTO_TEST_CASE(domain_propagation)
{
    {
        // [10,80] -- 38 --> [30,100]
        //    <------ -30 ----|
        // Expected after propagation
        // [10,70] -- 38 --> [48,100]
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,80));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(30,100));
        //point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(0,100));

        stn.addInterval(tp0, tp1, Bounds(30,38));

        // Using for loop to check stability of result
        for(int i = 0; i < 2; ++i)
        {
            graph_analysis::BaseGraph::Ptr baseGraph = stn.propagate();

            BOOST_REQUIRE_MESSAGE( tp0->getUpperBound() == 70, "Upper bound corrected after propagation: expected 70, actual " << tp0->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp1->getLowerBound() == 48, "Lower bound corrected after propagation: expected 48, actual " << tp1->getLowerBound());
        }


        // [10,80] -- 38 --> [30,100] -- 30 --> [70,100]
        //    <------ -30 ----|    <--- -10 ------|
        // Expected after propagation
        // [10,60] -- 38 --> [48,90] --- 30 --> [78,100]
        //    <------ -30 ----|    <--- -10 ------|
        point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(70,100));
        stn.addInterval(tp1,tp2, Bounds(10,30));
        // Using for loop to check stability of result
        for(int i = 0; i < 2; ++i)
        {
            graph_analysis::BaseGraph::Ptr baseGraph = stn.propagate();

            BOOST_REQUIRE_MESSAGE( tp0->getUpperBound() == 60, "Upper bound corrected after propagation: expected 70, actual " << tp0->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp1->getLowerBound() == 48, "Lower bound corrected after propagation: expected 48, actual " << tp1->getLowerBound());
            BOOST_REQUIRE_MESSAGE( tp1->getUpperBound() == 90, "Upper bound corrected after propagation: expected 90, actual " << tp1->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp2->getLowerBound() == 78, "Lower bound corrected after propagation: expected 78, actual " << tp2->getLowerBound());

            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-domain_propagation", *baseGraph, graph_analysis::representation::GEXF);
        }

    }
}

BOOST_AUTO_TEST_SUITE_END()
