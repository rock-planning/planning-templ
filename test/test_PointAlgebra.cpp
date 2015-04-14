#include <boost/test/unit_test.hpp>
#include <numeric/Combinatorics.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(point_algebra)

BOOST_AUTO_TEST_CASE(qualitative_constraints)
{
    using namespace templ::solvers::temporal::point_algebra;

    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(Universal, Equal), "Consistency and =" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(Universal, Less), "Consistency P and <" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(Universal, Greater), "Consistency P and >" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(Universal, GreaterOrEqual), "Consistency P and >= " );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(Universal, LessOrEqual), "Consistency: P and <=" );

    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(Equal, Equal), "Consistency = and =" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(Equal, LessOrEqual), "Consistency: = and <=" );
    BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(Equal, GreaterOrEqual), "Consistency: = and >=" );
    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(Equal, Less),    "Consistency: = and <" );
    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(Equal, Greater), "Consistency: = and >" );

    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(Less, LessOrEqual),       "Consistency: < and <=" );
    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(Less, GreaterOrEqual),    "Consistency: < and >=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(Less, Less),              "Consistency: < and <" );
    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(Less, Greater),           "Consistency: < and >" );

    BOOST_REQUIRE_MESSAGE(! QualitativeTimePointConstraint::isConsistent(Greater, LessOrEqual),       "Consistency: > and <=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(Greater, GreaterOrEqual),    "Consistency: > and >=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(Greater, Greater),           "Consistency: > and >" );

    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(LessOrEqual, GreaterOrEqual),    "Consistency: <= and >=" );
    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(LessOrEqual, LessOrEqual),       "Consistency: <= and <=" );

    BOOST_REQUIRE_MESSAGE(  QualitativeTimePointConstraint::isConsistent(GreaterOrEqual, GreaterOrEqual),       "Consistency: >= and >=" );

    std::vector<QualitativeConstraintType> types = QualitativeTimePointConstraint::getAllConstraintTypes();
    numeric::Combination<QualitativeConstraintType> combination(types, 2, numeric::EXACT);
    do {
        std::vector<QualitativeConstraintType> testPair = combination.current();
        BOOST_REQUIRE_MESSAGE( QualitativeTimePointConstraint::isConsistent(testPair[0], testPair[1]) == QualitativeTimePointConstraint::isConsistent(testPair[1], testPair[0]), "Check commutative constraint: " << QualitativeConstraintTypeTxt[ testPair[0] ] << " and " << QualitativeConstraintTypeTxt[ testPair[1] ] );

        try {
            BOOST_CHECK_MESSAGE( QualitativeTimePointConstraint::getComposition(testPair[0], testPair[1]) == QualitativeTimePointConstraint::getComposition(testPair[1], testPair[0]), "Check composition constraint: " << QualitativeConstraintTypeTxt[ testPair[0] ] << " and " << QualitativeConstraintTypeTxt[ testPair[1] ] );
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
