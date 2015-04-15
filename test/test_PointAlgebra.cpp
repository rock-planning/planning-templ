#include <boost/test/unit_test.hpp>
#include <numeric/Combinatorics.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

typedef point_algebra::QualitativeTimePointConstraint QTPC;

BOOST_AUTO_TEST_SUITE(point_algebra)

BOOST_AUTO_TEST_CASE(qualitative_constraints)
{
    using namespace templ::solvers::temporal::point_algebra;

    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::Equal), "Consistency and =" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::Less), "Consistency P and <" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::Greater), "Consistency P and >" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::GreaterOrEqual), "Consistency P and >= " );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(QTPC::Universal, QTPC::LessOrEqual), "Consistency: P and <=" );

    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(QTPC::Equal, QTPC::Equal), "Consistency = and =" );
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

BOOST_AUTO_TEST_CASE(timepoint_comparator)
{
}

BOOST_AUTO_TEST_SUITE_END()
