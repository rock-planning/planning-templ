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
// Input (a * near a vertex means that it is a target vertex):    
// .-------------------------------------------------------
// .                                                      .
// v0 -------[10,20] [30,40]---> v1                       .
// .                          .     .                     .
// .                       .          .                   .
//[20,130]             [0,40]       [20,40]            [30,110]
//[140,170]        [60,90]            [60,70]             .
//[180,200]    [130,150]                  .               .
// .         .                              .             .
// .     .                                    .           .
// *  *                                         *         .
// v3 <---------------[0,50]--------------------v2 <-------
//
//
//Expected result:
// .-------------------------------------------------------
// .                                                      .   
// v0 ------[10,20] [30,40]--> v1                         .
// .                          .  .                        .
// .                       .      .                       .
// .                [20,40]     [20,40]                [30,110]
//[30,130]       [60,90]          [60,70]                 .
// .            .                    .                    .
// .         .                        .                   .
// .     .                             .                  .
// * *                                  *                 .
// v3 <-----------[0,50]----------------v2 <---------------


    TemporalConstraintNetwork tcn,expected;

    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1));
    i0->addInterval(Bounds(10,20));
    i0->addInterval(Bounds(30,40));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v1,v2));
    i1->addInterval(Bounds(20,40));
    i1->addInterval(Bounds(60,70));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v2,v3));
    i2->addInterval(Bounds(0,50));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v1,v3));
    i3->addInterval(Bounds(0,40));
    i3->addInterval(Bounds(60,90));
    i3->addInterval(Bounds(130,150));
    IntervalConstraint::Ptr i4(new IntervalConstraint(v0,v3));
    i4->addInterval(Bounds(20,130));
    i4->addInterval(Bounds(140,170));
    i4->addInterval(Bounds(180,200));
    IntervalConstraint::Ptr i5(new IntervalConstraint(v0,v2));
    i5->addInterval(Bounds(30,110));

    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    tcn.addIntervalConstraint(i2);
    tcn.addIntervalConstraint(i3);
    tcn.addIntervalConstraint(i4);
    tcn.addIntervalConstraint(i5);

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 6,"Expected: 6, Actual: "<<tcn.getEdgeNumber());

    tcn.upperLowerTightening();
    BaseGraph::Ptr graph = tcn.getDistanceGraph();   

    IntervalConstraint::Ptr e0(new IntervalConstraint(v0,v1));
    e0->addInterval(Bounds(10,20));
    e0->addInterval(Bounds(30,40));
    IntervalConstraint::Ptr e1(new IntervalConstraint(v1,v2));
    e1->addInterval(Bounds(20,40));
    e1->addInterval(Bounds(60,70));
    IntervalConstraint::Ptr e2(new IntervalConstraint(v2,v3));
    e2->addInterval(Bounds(0,50));
    IntervalConstraint::Ptr e3(new IntervalConstraint(v1,v3));
    e3->addInterval(Bounds(20,40));
    e3->addInterval(Bounds(60,90));
    IntervalConstraint::Ptr e4(new IntervalConstraint(v0,v3));
    e4->addInterval(Bounds(30,130));
    IntervalConstraint::Ptr e5(new IntervalConstraint(v0,v2));
    e5->addInterval(Bounds(30,110));

    expected.addIntervalConstraint(e0);
    expected.addIntervalConstraint(e1);
    expected.addIntervalConstraint(e2);
    expected.addIntervalConstraint(e3);
    expected.addIntervalConstraint(e4); 
    expected.addIntervalConstraint(e5);

//  check if the expected result and the actual result are the same
    bool ok = expected.areEqual(graph);
    BOOST_REQUIRE_MESSAGE(ok, "Expected: 1, Actual: " << ok);
}

BOOST_AUTO_TEST_SUITE_END()