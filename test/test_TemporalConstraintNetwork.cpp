#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/WeightedEdge.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;
using namespace graph_analysis;

BOOST_AUTO_TEST_SUITE(temporal_constraint_network)


BOOST_AUTO_TEST_CASE(test_ult)
{

// Input:
// v0 -------------[10,40]-------v1
// .                          .     .
// .                       .          .
//[20,130]             [0,40]       [20,40]
//[140,170]        [60,90]            [60,70]
//[180,200]    [130,150]                  .
// .         .                              .
// .     .                                    .
// . .                                          .
// v3 ----------------[0,50]--------------------v2
//
//
//Expected result:
// v0 -------------[10,40]-----v1
// .                          .  .
// .                       .      .
// .                [20,40]     [20,40]
//[30,130]       [60,90]          [60,70]
// .            .                    .
// .         .                        .
// .     .                             .
// . .                                  .
// v3 ------------[0,50]----------------v2 


    TemporalConstraintNetwork tcn;

    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1,10,40));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v1,v2,20,40));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v1,v2,60,70));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v2,v3,0,50));
    IntervalConstraint::Ptr i4(new IntervalConstraint(v1,v3,0,40));
    IntervalConstraint::Ptr i5(new IntervalConstraint(v1,v3,60,90));
    IntervalConstraint::Ptr i6(new IntervalConstraint(v1,v3,130,150));
    IntervalConstraint::Ptr i7(new IntervalConstraint(v0,v3,20,130));
    IntervalConstraint::Ptr i8(new IntervalConstraint(v0,v3,140,170));
    IntervalConstraint::Ptr i9(new IntervalConstraint(v0,v3,180,200));

    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    tcn.addIntervalConstraint(i2);
    tcn.addIntervalConstraint(i3);
    tcn.addIntervalConstraint(i4);
    tcn.addIntervalConstraint(i5);
    tcn.addIntervalConstraint(i6);
    tcn.addIntervalConstraint(i7);
    tcn.addIntervalConstraint(i8);
    tcn.addIntervalConstraint(i9);
    

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 10,"Expected: 10, Actual: "<<tcn.getEdgeNumber());

    tcn.upperLowerTightening();
    
    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 7,"Expected: 7, Actual: "<<tcn.getEdgeNumber());
}

BOOST_AUTO_TEST_SUITE_END()