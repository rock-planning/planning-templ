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

BOOST_AUTO_TEST_CASE(composition)
{
    using namespace templ::solvers::temporal::point_algebra;

    base::Time start = base::Time::now();
    int i = 0;
    for(; i < 1000; ++i)
    {
        QTPC::Type type = QualitativeTimePointConstraint::getComposition(QTPC::Greater, QTPC::GreaterOrEqual);
    }
    base::Time stop = base::Time::now();

    BOOST_TEST_MESSAGE("getComposition requires: " << (stop-start).toSeconds()/(i*1.0) << "s/call");
}

BOOST_AUTO_TEST_CASE(symmetric)
{
    using namespace templ::solvers::temporal::point_algebra;

    BOOST_REQUIRE_MESSAGE(QualitativeTimePointConstraint::getSymmetric(QTPC::Greater) == QualitativeTimePointConstraint::Less, "Greater symmetric to less");
    BOOST_REQUIRE_MESSAGE(QualitativeTimePointConstraint::getSymmetric(QTPC::Less) == QualitativeTimePointConstraint::Greater, "Greater symmetric to less");

    base::Time start = base::Time::now();
    int i = 0;
    for(; i < 1000; ++i)
    {
        QTPC::Type type = QualitativeTimePointConstraint::getSymmetric(QTPC::Greater);
    }
    base::Time stop = base::Time::now();
    BOOST_TEST_MESSAGE("getSymmetry requires: " << (stop-start).toSeconds()/(i*1.0) << "s/call");
}

BOOST_AUTO_TEST_CASE(intersection)
{
    using namespace templ::solvers::temporal::point_algebra;

    base::Time start = base::Time::now();
    int i = 0;
    for(; i < 1000; ++i)
    {
        QTPC::Type type = QualitativeTimePointConstraint::getIntersection(QTPC::GreaterOrEqual, QTPC::Equal);
    }
    base::Time stop = base::Time::now();

    BOOST_TEST_MESSAGE("getIntersection requires: " << (stop-start).toSeconds()/(i*1.0) << "s/call");
}


BOOST_AUTO_TEST_CASE(bidirectional_constraint_type)
{
    using namespace templ::solvers::temporal::point_algebra;
    using namespace graph_analysis;

    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());
    {
        QualitativeTimePoint::Ptr lastTp;
        int i = 0;
        for(; i < 100; ++i)
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
    }

    base::Time start = base::Time::now();
    std::vector<Vertex::Ptr> vertices = tcn->getGraph()->getAllVertices();

    Vertex::Ptr source = vertices[10];
    Vertex::Ptr target = vertices[50];

    int i = 0;
    for(; i < 1000; ++i)
    {
        QTPC::Type type = tcn->getBidirectionalConstraintType(source, target);
    }
    base::Time stop = base::Time::now();

    BOOST_TEST_MESSAGE("getBidirectionalConstraintType requires: " << (stop-start).toSeconds()/(i*1.0) << "s/call");
}



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
        BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(testPair[0], testPair[1]) == QualitativeTimePointConstraint::isConsistent(testPair[1], testPair[0]), "Check commutative constraint: " << QualitativeTimePointConstraint::TypeSymbol[ testPair[0] ] << " and " << QualitativeTimePointConstraint::TypeSymbol[ testPair[1] ] );

        try {
            BOOST_CHECK_MESSAGE( QualitativeTimePointConstraint::getComposition(testPair[0], testPair[1]) == QualitativeTimePointConstraint::getComposition(testPair[1], testPair[0]), "Check composition constraint: " << QualitativeTimePointConstraint::TypeSymbol[ testPair[0] ] << " and " << QualitativeTimePointConstraint::TypeSymbol[ testPair[1] ] );
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

BOOST_AUTO_TEST_CASE(complete)
{
    using namespace templ::solvers::temporal::point_algebra;

    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr lastTp;
    int i = 0;
    for(; i < 5; ++i)
    {
        std::stringstream ss;
        ss << "t" << i;
        QualitativeTimePoint::Ptr tp(new QualitativeTimePoint(ss.str()));
        if(lastTp)
        {
            tcn->addQualitativeConstraint(lastTp, tp, QTPC::Greater);
        }

        lastTp = tp;
    }
    BOOST_TEST_MESSAGE("Network constructed");

    {
        base::Time start = base::Time::now();
        bool consistent = tcn->isConsistent();
        base::Time stop = base::Time::now() - start;
        BOOST_REQUIRE_MESSAGE(consistent, "1. Temporal constraint network is consistent: time " << stop.toSeconds() );
    }
    {
        base::Time start = base::Time::now();
        bool consistent = tcn->isConsistent();
        base::Time stop = base::Time::now() - start;
        BOOST_REQUIRE_MESSAGE(consistent, "2. Temporal constraint network is consistent: time " << stop.toSeconds() );
    }
    tcn->save("/tmp/thomas-test-network");

    QualitativeTimePoint::Ptr t0(new QualitativeTimePoint("t0"));
    QualitativeTimePoint::Ptr t1(new QualitativeTimePoint("t1"));
    QualitativeTimePoint::Ptr t2(new QualitativeTimePoint("t2"));

    tcn->addQualitativeConstraint(t0, t1, QTPC::Greater);
    tcn->addQualitativeConstraint(t1, t2, QTPC::Greater);
    tcn->addQualitativeConstraint(t2, t1, QTPC::Greater);

    BOOST_REQUIRE_THROW(tcn->incrementalPathConsistency(), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(consistency_check_performance)
{
    using namespace templ::solvers::temporal::point_algebra;

    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr lastTp;
    int i = 0;
    for(; i < 100; ++i)
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

BOOST_AUTO_TEST_CASE(inconsistency_0)
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
    tcn->save("/tmp/templ-test-point_algebra-inconsistency_0-test");
    BOOST_REQUIRE_MESSAGE(!consistent, "Temporal constraint network is inconsistent");
}

BOOST_AUTO_TEST_CASE(inconsistency_1)
{
    using namespace templ::solvers::temporal::point_algebra;
    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr t0(new QualitativeTimePoint("t0"));
    QualitativeTimePoint::Ptr t1(new QualitativeTimePoint("t1"));
    QualitativeTimePoint::Ptr t2(new QualitativeTimePoint("t2"));
    QualitativeTimePoint::Ptr t3(new QualitativeTimePoint("t3"));
    QualitativeTimePoint::Ptr t4(new QualitativeTimePoint("t4"));
    QualitativeTimePoint::Ptr t5(new QualitativeTimePoint("t5"));
    QualitativeTimePoint::Ptr t6(new QualitativeTimePoint("t6"));
    QualitativeTimePoint::Ptr t7(new QualitativeTimePoint("t7"));
    QualitativeTimePoint::Ptr t8(new QualitativeTimePoint("t8"));

    tcn->addQualitativeConstraint(t1, t0, QTPC::Greater);
    tcn->addQualitativeConstraint(t2, t1, QTPC::Greater);
    tcn->addQualitativeConstraint(t3, t2, QTPC::Greater);
    tcn->addQualitativeConstraint(t4, t3, QTPC::Greater);
    tcn->addQualitativeConstraint(t5, t4, QTPC::Greater);
    tcn->addQualitativeConstraint(t6, t5, QTPC::Greater);
    tcn->addQualitativeConstraint(t7, t6, QTPC::Greater);
    tcn->addQualitativeConstraint(t8, t7, QTPC::Greater);
    tcn->addQualitativeConstraint(t5, t1, QTPC::Less);

    tcn->save("/tmp/test-point_algebra-consistency-test-pre");


    bool consistent = tcn->isConsistent();
    BOOST_REQUIRE_MESSAGE(!consistent, "Temporal constraint network is inconsistent");
}

BOOST_AUTO_TEST_CASE(inconsistency_benchmark)
{
    using namespace templ::solvers::temporal::point_algebra;
    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr t0(new QualitativeTimePoint("t0"));
    QualitativeTimePoint::Ptr t1(new QualitativeTimePoint("t1"));
    QualitativeTimePoint::Ptr t2(new QualitativeTimePoint("t2"));
    QualitativeTimePoint::Ptr t3(new QualitativeTimePoint("t3"));
    QualitativeTimePoint::Ptr t4(new QualitativeTimePoint("t4"));
    QualitativeTimePoint::Ptr t5(new QualitativeTimePoint("t5"));
    QualitativeTimePoint::Ptr t6(new QualitativeTimePoint("t6"));
    QualitativeTimePoint::Ptr t7(new QualitativeTimePoint("t7"));
    QualitativeTimePoint::Ptr t8(new QualitativeTimePoint("t8"));

    tcn->addQualitativeConstraint(t1, t0, QTPC::Greater);
    tcn->addQualitativeConstraint(t2, t1, QTPC::Greater);
    tcn->addQualitativeConstraint(t3, t2, QTPC::Greater);
    tcn->addQualitativeConstraint(t4, t3, QTPC::Greater);
    tcn->addQualitativeConstraint(t5, t4, QTPC::Greater);
    tcn->addQualitativeConstraint(t6, t5, QTPC::Greater);
    tcn->addQualitativeConstraint(t7, t6, QTPC::Greater);
    tcn->addQualitativeConstraint(t8, t7, QTPC::Greater);
    tcn->addQualitativeConstraint(t5, t1, QTPC::Less);

    tcn->save("/tmp/test-point_algebra-consistency-test-pre");

    bool consistent = tcn->isConsistent();
    BOOST_REQUIRE_MESSAGE(!consistent, "Temporal constraint network is inconsistent");
}

BOOST_AUTO_TEST_SUITE_END()
