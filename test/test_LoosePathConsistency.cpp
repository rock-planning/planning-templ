#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/LoosePathConsistency.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;
using namespace graph_analysis;

BOOST_AUTO_TEST_SUITE(loose_path_consistency)


BOOST_AUTO_TEST_CASE(test_intersectionNetwork)
{
    TemporalConstraintNetwork tcn,expected;

    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1));
    i0->addInterval(Bounds(10,20));
    i0->addInterval(Bounds(110,110));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v1,v2));
    i1->addInterval(Bounds(20,30));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v2,v3));
    i2->addInterval(Bounds(110,120));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v1,v3));
    i3->addInterval(Bounds(130,140));
    IntervalConstraint::Ptr i4(new IntervalConstraint(v0,v3));
    i4->addInterval(Bounds(150,160));
    IntervalConstraint::Ptr i5(new IntervalConstraint(v0,v2));
    i5->addInterval(Bounds(30,140));

    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    tcn.addIntervalConstraint(i2);
    tcn.addIntervalConstraint(i3);
    tcn.addIntervalConstraint(i4);
    tcn.addIntervalConstraint(i5);

    IntervalConstraint::Ptr e0(new IntervalConstraint(v0,v1));
    e0->addInterval(Bounds(10,30));
    IntervalConstraint::Ptr e1(new IntervalConstraint(v1,v2));
    e1->addInterval(Bounds(10,30));
    IntervalConstraint::Ptr e2(new IntervalConstraint(v2,v3));
    e2->addInterval(Bounds(100,120));
    IntervalConstraint::Ptr e3(new IntervalConstraint(v1,v3));
    e3->addInterval(Bounds(130,150));
    IntervalConstraint::Ptr e4(new IntervalConstraint(v0,v3));
    e4->addInterval(Bounds(140,160));
    e4->addInterval(Bounds(240,250));
    IntervalConstraint::Ptr e5(new IntervalConstraint(v0,v2));
    e5->addInterval(Bounds(30,50));

    expected.addIntervalConstraint(e0);
    expected.addIntervalConstraint(e1);
    expected.addIntervalConstraint(e2);
    expected.addIntervalConstraint(e3);
    expected.addIntervalConstraint(e4);
    expected.addIntervalConstraint(e5);

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 6,"Expected: 6, Actual: "<<tcn.getEdgeNumber());

    TemporalConstraintNetwork result = LoosePathConsistency::intersectionNetwork(tcn);
    BaseGraph::Ptr graph = result.getDistanceGraph();

    BOOST_REQUIRE_MESSAGE(expected.areEqual(graph), "Expected: 1, Actual: "<<expected.areEqual(graph));
}

BOOST_AUTO_TEST_CASE(test_looseIntersection)
{
    TemporalConstraintNetwork tcn,tcn2,expected;

    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1));
    i0->addInterval(Bounds(10,30));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v1,v2));
    i1->addInterval(Bounds(10,30));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v2,v3));
    i2->addInterval(Bounds(100,120));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v1,v3));
    i3->addInterval(Bounds(130,150));
    IntervalConstraint::Ptr i4(new IntervalConstraint(v0,v3));
    i4->addInterval(Bounds(140,160));
    i4->addInterval(Bounds(240,250));
    IntervalConstraint::Ptr i5(new IntervalConstraint(v0,v2));
    i5->addInterval(Bounds(30,50));

    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    tcn.addIntervalConstraint(i2);
    tcn.addIntervalConstraint(i3);
    tcn.addIntervalConstraint(i4);
    tcn.addIntervalConstraint(i5);

    IntervalConstraint::Ptr p0(new IntervalConstraint(v0,v1));
    p0->addInterval(Bounds(10,20));
    p0->addInterval(Bounds(110,110));
    IntervalConstraint::Ptr p1(new IntervalConstraint(v1,v2));
    p1->addInterval(Bounds(20,30));
    IntervalConstraint::Ptr p2(new IntervalConstraint(v2,v3));
    p2->addInterval(Bounds(110,120));
    IntervalConstraint::Ptr p3(new IntervalConstraint(v1,v3));
    p3->addInterval(Bounds(130,140));
    IntervalConstraint::Ptr p4(new IntervalConstraint(v0,v3));
    p4->addInterval(Bounds(150,160));
    IntervalConstraint::Ptr p5(new IntervalConstraint(v0,v2));
    p5->addInterval(Bounds(30,140));

    tcn2.addIntervalConstraint(p0);
    tcn2.addIntervalConstraint(p1);
    tcn2.addIntervalConstraint(p2);
    tcn2.addIntervalConstraint(p3);
    tcn2.addIntervalConstraint(p4);
    tcn2.addIntervalConstraint(p5);

    IntervalConstraint::Ptr e0(new IntervalConstraint(v0,v1));
    e0->addInterval(Bounds(10,20));
    IntervalConstraint::Ptr e1(new IntervalConstraint(v1,v2));
    e1->addInterval(Bounds(20,30));
    IntervalConstraint::Ptr e2(new IntervalConstraint(v2,v3));
    e2->addInterval(Bounds(110,120));
    IntervalConstraint::Ptr e3(new IntervalConstraint(v1,v3));
    e3->addInterval(Bounds(130,140));
    IntervalConstraint::Ptr e4(new IntervalConstraint(v0,v3));
    e4->addInterval(Bounds(150,160));
    IntervalConstraint::Ptr e5(new IntervalConstraint(v0,v2));
    e5->addInterval(Bounds(30,50));

    expected.addIntervalConstraint(e0);
    expected.addIntervalConstraint(e1);
    expected.addIntervalConstraint(e2);
    expected.addIntervalConstraint(e3);
    expected.addIntervalConstraint(e4);
    expected.addIntervalConstraint(e5);


    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 6,"Expected: 6, Actual: "<<tcn.getEdgeNumber());

    TemporalConstraintNetwork result = LoosePathConsistency::looseNetwork(tcn,tcn2);
    BaseGraph::Ptr graph = result.getDistanceGraph();

    BOOST_REQUIRE_MESSAGE(expected.areEqual(graph), "Expected: 1, Actual: "<<expected.areEqual(graph));
}

/*
INPUT:
    v1 --[0,1] [16,17] [23,24]----> v2
    ^                           ^
    .                   [34,50]       
    .                [23,33]
  [1,2]           [0,22]
  [11,12]        .
  [21,22]     .
    .      . 
    .   .  
    v0

EXPECTED (after execution of loose path-consistency): 
    v1 --[0,1] [16,17] [23,24]----> v2
    ^                           ^
    .                   [34,46]       
    .                [24,29]
  [1,2]           [1,22]
  [11,12]        .
  [21,22]     .
    .      . 
    .   .  
    v0

*/
BOOST_AUTO_TEST_CASE(test_lpc)
{
    TemporalConstraintNetwork tcn,expected;

    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1));
    i0->addInterval(Bounds(1,2));
    i0->addInterval(Bounds(11,12));
    i0->addInterval(Bounds(21,22));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v1,v2));
    i1->addInterval(Bounds(0,1));
    i1->addInterval(Bounds(16,17));
    i1->addInterval(Bounds(23,24));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v0,v2));
    i2->addInterval(Bounds(0,22));
    i2->addInterval(Bounds(23,33));
    i2->addInterval(Bounds(34,50));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v0,v2));
    i3->addInterval(Bounds(1,22));
    i3->addInterval(Bounds(24,29));
    i3->addInterval(Bounds(34,46));

    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    tcn.addIntervalConstraint(i2);

    expected.addIntervalConstraint(i0);
    expected.addIntervalConstraint(i1);
    expected.addIntervalConstraint(i3);

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 3,"Expected: 3, Actual: "<<tcn.getEdgeNumber());

    TemporalConstraintNetwork result = LoosePathConsistency::loosePathConsistency(tcn);
    BaseGraph::Ptr graph = result.getDistanceGraph();

    BOOST_REQUIRE_MESSAGE(expected.areEqual(graph),"Expected: 1, Actual: "<<expected.areEqual(graph));
}

BOOST_AUTO_TEST_SUITE_END()