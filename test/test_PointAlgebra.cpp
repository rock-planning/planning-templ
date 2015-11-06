#include <boost/test/unit_test.hpp>
#include <base/Time.hpp>
#include <numeric/Combinatorics.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

typedef point_algebra::QualitativeTimePointConstraint QTPC;

BOOST_AUTO_TEST_SUITE(point_algebra)

BOOST_AUTO_TEST_CASE(qualitative_constraints)
{
    using namespace templ::solvers::temporal::point_algebra;

    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::Equal), "Consistency: P and =" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::Less), "Consistency: P and <" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::Greater), "Consistency: P and >" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::GreaterOrEqual), "Consistency: P and >= " );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::LessOrEqual), "Consistency: P and <=" );

    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::Equal, QTPC::Equal), "Consistency: = and =" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::Equal, QTPC::LessOrEqual), "Consistency: = and <=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::Equal, QTPC::GreaterOrEqual), "Consistency: = and >=" );
    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(QTPC::Equal, QTPC::Less),    "Consistency: = and <" );
    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(QTPC::Equal, QTPC::Greater), "Consistency: = and >" );

    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::Less, QTPC::LessOrEqual),       "Consistency: < and <=" );
    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(QTPC::Less, QTPC::GreaterOrEqual),    "Consistency: < and >=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::Less, QTPC::Less),              "Consistency: < and <" );
    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(QTPC::Less, QTPC::Greater),           "Consistency: < and >" );

    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(QTPC::Greater, QTPC::LessOrEqual),       "Consistency: > and <=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::Greater, QTPC::GreaterOrEqual),    "Consistency: > and >=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::Greater, QTPC::Greater),           "Consistency: > and >" );

    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::LessOrEqual, QTPC::GreaterOrEqual),    "Consistency: <= and >=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::LessOrEqual, QTPC::LessOrEqual),       "Consistency: <= and <=" );

    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::GreaterOrEqual, QTPC::GreaterOrEqual),       "Consistency: >= and >=" );

    std::vector<QualitativeTimePointConstraint::Type> types = QualitativeTimePointConstraint::getAllConstraintTypes();
    numeric::Combination<QualitativeTimePointConstraint::Type> combination(types, 2, numeric::EXACT);
    do {
        std::vector<QualitativeTimePointConstraint::Type> testPair = combination.current();
        BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(testPair[0], testPair[1]) == QualitativeTimePointConstraint::isConsistent(testPair[1], testPair[0]), "Check commutative constraint: " << QualitativeTimePointConstraint::TypeTxt[ testPair[0] ] << " and " << QualitativeTimePointConstraint::TypeTxt[ testPair[1] ] );

        try {
            BOOST_CHECK_MESSAGE( QualitativeTimePointConstraint::getComposition(testPair[0], testPair[1]) == QualitativeTimePointConstraint::getComposition(testPair[1], testPair[0]), "Check composition constraint: " << QualitativeTimePointConstraint::TypeTxt[ testPair[0] ] << " and " << QualitativeTimePointConstraint::TypeTxt[ testPair[1] ] );
        } catch(const std::runtime_error& e)
        {
            BOOST_TEST_MESSAGE(e.what());
        }

    } while(combination.next());
}

BOOST_AUTO_TEST_CASE(qualitative_timepoints)
{
    using namespace templ::solvers::temporal::point_algebra;
    QualitativeTimePoint tp0("tp0");
    QualitativeTimePoint tp1("tp1");
    QualitativeTimePoint tp2("tp2");

    BOOST_REQUIRE_MESSAGE(tp0 != tp1, "Timepoints tp0 and tp1 are different");
    BOOST_REQUIRE_MESSAGE(tp0 != tp2, "Timepoints tp0 and tp2 are different");
    BOOST_REQUIRE_MESSAGE(tp1 != tp2, "Timepoints tp1 and tp2 are different");

    tp1.addAlias("tp3");
    tp2.addAlias("tp3");

    BOOST_REQUIRE_MESSAGE(tp0 != tp1, "Timepoints tp0 and tp1 are different");
    BOOST_REQUIRE_MESSAGE(tp0 != tp2, "Timepoints tp0 and tp2 are different");
    BOOST_REQUIRE_MESSAGE(tp1 == tp2, "Timepoints tp1 and tp2 are same due to same alias 'tp3'");
}

BOOST_AUTO_TEST_CASE(consistency_check_performance)
{
    using namespace templ::solvers::temporal::point_algebra;

    TemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr lastTp;
    int i = 0;
    for(; i < 10; ++i)
    {
        std::stringstream ss;
        ss << "t" << i;
        QualitativeTimePoint::Ptr tp(new QualitativeTimePoint(ss.str()));
        if(lastTp)
        {
            tcn->addQualitativeConstraint(lastTp, tp, QTPC::Greater);
            tcn->addQualitativeConstraint(tp, lastTp, QTPC::Less);
        }

        lastTp = tp;
    }

    {
        base::Time start = base::Time::now();
        bool consistent = tcn->isConsistent();
        base::Time stop = base::Time::now() - start;
        BOOST_REQUIRE_MESSAGE(consistent, "Temporal constraint network with " << i << " timepoints is consistent: computing time: " << stop.toSeconds());
    }
}

BOOST_AUTO_TEST_CASE(inconsistency)
{
    using namespace templ::solvers::temporal::point_algebra;
    TemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr t0(new QualitativeTimePoint("t0"));
    QualitativeTimePoint::Ptr t1(new QualitativeTimePoint("t1"));
    QualitativeTimePoint::Ptr t2(new QualitativeTimePoint("t2"));

    tcn->addQualitativeConstraint(t0, t1, QTPC::Greater);
    tcn->addQualitativeConstraint(t1, t2, QTPC::Greater);
    tcn->addQualitativeConstraint(t2, t1, QTPC::Greater);
    
    bool consistent = tcn->isConsistent();
    tcn->save("/tmp/test-point_algebra-inconsistency-test");
    BOOST_REQUIRE_MESSAGE(!consistent, "Temporal constraint network is inconsistent");
}

BOOST_AUTO_TEST_SUITE_END()
