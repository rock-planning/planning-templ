#include <boost/test/unit_test.hpp>
#include <terep/solvers/temporal/SimpleTemporalNetwork.hpp>
#include <numeric/Combinatorics.hpp>

using namespace terep::solvers;
using namespace terep::solvers::temporal;

BOOST_AUTO_TEST_SUITE(simple_temporal_network)

BOOST_AUTO_TEST_CASE(point_algebra_test)
{
    using namespace point_algebra;

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

BOOST_AUTO_TEST_CASE(consistency_checking)
{
    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(0,100));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(0,100));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(0,100));

        stn.addQualitativeConstraint(tp0, tp1, point_algebra::GreaterOrEqual);
        stn.addQualitativeConstraint(tp1, tp2, point_algebra::GreaterOrEqual);

        BOOST_REQUIRE_MESSAGE(stn.isConsistent(), "STN is consistent");
    }
    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(0,100));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(0,100));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(0,100));

        stn.addQualitativeConstraint(tp0, tp1, point_algebra::GreaterOrEqual);
        stn.addQualitativeConstraint(tp1, tp2, point_algebra::GreaterOrEqual);
        stn.addQualitativeConstraint(tp0, tp2, point_algebra::Less);

        BOOST_REQUIRE_MESSAGE(!stn.isConsistent(), "STN is inconsistent");
    }
    {
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(0,100));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(0,100));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(0,100));

        stn.addQualitativeConstraint(tp0, tp1, point_algebra::Greater);
        stn.addQualitativeConstraint(tp1, tp2, point_algebra::Less);
        stn.addQualitativeConstraint(tp0,tp2, point_algebra::Less);

        BOOST_REQUIRE_MESSAGE(stn.isConsistent(), "STN is consistent");
    }
}

BOOST_AUTO_TEST_SUITE_END()
