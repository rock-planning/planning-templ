#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/LoosePathConsistency.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;
using namespace graph_analysis;

BOOST_AUTO_TEST_SUITE(loose_path_consistency)

BOOST_AUTO_TEST_CASE(operation_intersection)
{
    Bounds::List a;
    a.push_back(Bounds(1,4));
    a.push_back(Bounds(10,15));

    Bounds::List b;
    b.push_back(Bounds(3,11));
    b.push_back(Bounds(14,19));

    {
        Bounds::List intersection = LoosePathConsistency::intersection(a, b);

        Bounds::List expectedIntersection;
        expectedIntersection.push_back(Bounds(3,4));
        expectedIntersection.push_back(Bounds(10,11));
        expectedIntersection.push_back(Bounds(14,15));

        BOOST_REQUIRE_MESSAGE(expectedIntersection == intersection, "Expected intersection: " << Bounds::toString(expectedIntersection) << " got " << Bounds::toString(intersection));

    }

    {
        Bounds::List looseIntersection = LoosePathConsistency::looseIntersection(a, b);
        BOOST_REQUIRE_MESSAGE(looseIntersection.size() == 2, "Bounds loose looseIntersection should be of size 2, but was " << looseIntersection.size());

        Bounds::List expectedIntersection;
        expectedIntersection.push_back(Bounds(3,4));
        expectedIntersection.push_back(Bounds(10,15));

        BOOST_REQUIRE_MESSAGE(expectedIntersection == looseIntersection, "Expected looseIntersection: " << Bounds::toString(expectedIntersection) << " got " << Bounds::toString(looseIntersection));
    }
}

BOOST_AUTO_TEST_CASE(operation_composition)
{
    {
        Bounds::List a;
        a.push_back(Bounds(-1.25,0.25));
        a.push_back(Bounds(2.75, 4.25));

        Bounds::List b;
        b.push_back(Bounds(-0.25,1.25));
        b.push_back(Bounds(3.75,4.25));

        {
            Bounds::List composition = LoosePathConsistency::composition(a, b);
            Bounds::List expectedComposition;
            expectedComposition.push_back(Bounds(-1.50,1.50));
            expectedComposition.push_back(Bounds(2.50,5.50));
            expectedComposition.push_back(Bounds(6.50,8.50));

            BOOST_REQUIRE_MESSAGE(expectedComposition == composition, "Expected composition: " << Bounds::toString(expectedComposition) << " got " << Bounds::toString(composition));
        }
    }

    {
        Bounds::List a;
        a.push_back(Bounds(-100, -80));

        Bounds::List b;
        b.push_back(Bounds(10, 120));

        Bounds::List composition = LoosePathConsistency::composition(a,b);
        Bounds::List expectedComposition;
        expectedComposition.push_back(Bounds(-90,40));
        BOOST_REQUIRE_MESSAGE(expectedComposition == composition, "Expected composition: " << Bounds::toString(expectedComposition) << " got " << Bounds::toString(composition));
    }

    {
        Bounds::List a;
        a.push_back(Bounds(80, 100));
        a.push_back(Bounds(150, 160));
        a.push_back(Bounds(180, 190));

        Bounds::List b;
        b.push_back(Bounds(-40, -30));
        // -150 is wrong, but must be -140 to come to the require results
        b.push_back(Bounds(-140, -130));

        Bounds::List composition = LoosePathConsistency::composition(a,b);
        Bounds::List expectedComposition;
        expectedComposition.push_back(Bounds(-60,-30));
        expectedComposition.push_back(Bounds(10,30));
        expectedComposition.push_back(Bounds(40,70));
        expectedComposition.push_back(Bounds(110,130));
        expectedComposition.push_back(Bounds(140,160));

        BOOST_REQUIRE_MESSAGE(expectedComposition == composition, "Expected composition: " << Bounds::toString(expectedComposition) << " got " << Bounds::toString(composition));
    }

    {
        Bounds::List a;
        a.push_back(Bounds(20, 40));
        a.push_back(Bounds(100, 130));

        Bounds::List b;
        b.push_back(Bounds(50,70));
        b.push_back(Bounds(110,120));
        b.push_back(Bounds(130,140));
        b.push_back(Bounds(160,190));

        Bounds::List composition = LoosePathConsistency::composition(a,b);
        Bounds::List expectedComposition;
        // Commenting out which will be merged
        // a[0] intersect with b
        expectedComposition.push_back(Bounds(70,110));
        //expectedComposition.push_back(Bounds(130,160));
        //expectedComposition.push_back(Bounds(150,180));
        //expectedComposition.push_back(Bounds(180,230));

        // a[1] intersect with b
        //expectedComposition.push_back(Bounds(150,200));
        //expectedComposition.push_back(Bounds(210,250));
        //expectedComposition.push_back(Bounds(230,270));
        //expectedComposition.push_back(Bounds(260,320));

        // Intermediate intersections by merging overlapping
        //expectedComposition.push_back(Bounds(130,200));
        //expectedComposition.push_back(Bounds(210,320));
        //
        // Final intersection
        expectedComposition.push_back(Bounds(130,320));

        BOOST_REQUIRE_MESSAGE(expectedComposition == composition, "Expected composition: " << Bounds::toString(expectedComposition) << " got " << Bounds::toString(composition));
    }
}

BOOST_AUTO_TEST_CASE(operation_compute_disjoint_intervals)
{
    Bounds::List a;
    a.push_back(Bounds(0,10));
    a.push_back(Bounds(9,10));
    a.push_back(Bounds(9,20));
    a.push_back(Bounds(23,30));

    Bounds::List expected;
    expected.push_back(Bounds(0,20));
    expected.push_back(Bounds(23,30));

    Bounds::List result = LoosePathConsistency::computeDisjointIntervals(a);
    BOOST_REQUIRE_MESSAGE(expected == result, "Expected interval list: " << Bounds::toString(expected)
            << " got " << Bounds::toString(result) << "(size: " << result.size() << ")");
}

BOOST_AUTO_TEST_CASE(operation_loose_path)
{
    {
        Bounds::List a0_1;
        a0_1.push_back(Bounds(10,20));
        a0_1.push_back(Bounds(100,110));

        Bounds::List a1_2;
        a1_2.push_back(Bounds(20,40));
        a1_2.push_back(Bounds(100,110));

        // Check triangle a0_1 with completing vertex 2
        // empty constraint between 0 and 2, so composition must be empty
        Bounds::List composition = LoosePathConsistency::composition(a0_1, Bounds::List());
        Bounds::List looseIntersection = LoosePathConsistency::looseIntersection(a0_1, composition);

        Bounds::List expectedLooseIntersection;

        BOOST_REQUIRE_MESSAGE(looseIntersection == expectedLooseIntersection, "Expected loose intersection: " << Bounds::toString(expectedLooseIntersection) << " got " << Bounds::toString(looseIntersection) );


        Bounds::List a1_0 = Bounds::reverse(a0_1);
        Bounds::List a2_1 = Bounds::reverse(a1_2);

        {
            Bounds::List composition = LoosePathConsistency::composition(a1_0, Bounds::List());
            Bounds::List looseIntersection = LoosePathConsistency::looseIntersection(a1_0, composition);

            BOOST_REQUIRE_MESSAGE(looseIntersection == expectedLooseIntersection, "Expected loose intersection: " << Bounds::toString(expectedLooseIntersection) << " got " << Bounds::toString(looseIntersection) );
        }
    }
    {
        Bounds::List a0_1;
        a0_1.push_back(Bounds(10,20));
        a0_1.push_back(Bounds(100,110));
        Bounds::List a1_0 = Bounds::reverse(a0_1);

        Bounds::List a1_3;
        a1_3.push_back(Bounds(30,40));
        a1_3.push_back(Bounds(130,150));
        Bounds::List a3_1 = Bounds::reverse(a1_3);

        //a3_1.push_back(Bounds(-40, -30));
        //a3_1.push_back(Bounds(-150, -130));

        Bounds::List a0_3;
        a0_3.push_back(Bounds(80,100));
        a0_3.push_back(Bounds(150,160));
        a0_3.push_back(Bounds(180,190));
        Bounds::List a3_0 = Bounds::reverse(a0_3);

        {
            // Check triangle a0_1 with completing vertex 3
            Bounds::List composition = LoosePathConsistency::composition(a0_3, a3_1);
            BOOST_TEST_MESSAGE("Composition: " << Bounds::toString(composition) );

            Bounds::List intersection = LoosePathConsistency::intersection(a0_1, composition);
            BOOST_TEST_MESSAGE("Intersection: " << Bounds::toString(intersection) );

            Bounds::List looseIntersection = LoosePathConsistency::looseIntersection(a0_1, composition);

            Bounds::List expectedLooseIntersection;
            expectedLooseIntersection.push_back(Bounds(10,20));
            expectedLooseIntersection.push_back(Bounds(110,110));

            BOOST_REQUIRE_MESSAGE(looseIntersection == expectedLooseIntersection, "Expected loose intersection: " << Bounds::toString(expectedLooseIntersection) << " got " << Bounds::toString(looseIntersection) );
        }
        // check reverse
        {
            Bounds::List composition = LoosePathConsistency::composition(a1_3, a3_0);
            BOOST_TEST_MESSAGE("Composition: " << Bounds::toString(composition) );

            Bounds::List intersection = LoosePathConsistency::intersection(a1_0, composition);
            BOOST_TEST_MESSAGE("Intersection: " << Bounds::toString(intersection) );

            Bounds::List looseIntersection = LoosePathConsistency::looseIntersection(a1_0, composition);

            Bounds::List expectedLooseIntersection;
            expectedLooseIntersection.push_back(Bounds(-20,-10));
            expectedLooseIntersection.push_back(Bounds(-110,-110));

            BOOST_REQUIRE_MESSAGE(looseIntersection == expectedLooseIntersection, "Reverse check: expected loose intersection: " << Bounds::toString(expectedLooseIntersection) << " got " << Bounds::toString(looseIntersection) );
        }


        {
            Bounds::List composition = LoosePathConsistency::composition(a0_1, a1_3);
            BOOST_TEST_MESSAGE("Composition: " << Bounds::toString(composition) );
            Bounds::List looseIntersection = LoosePathConsistency::looseIntersection(a0_3, composition);

            Bounds::List expectedLooseIntersection;
            expectedLooseIntersection.push_back(Bounds(150,160));

            BOOST_REQUIRE_MESSAGE(looseIntersection == expectedLooseIntersection, "Expected loose intersection: " << Bounds::toString(expectedLooseIntersection) << " got " << Bounds::toString(looseIntersection) );
        }

    }

}

/**
 *       x0   -- [10,20] [100,110]   --> X1
 *       |                          /
 *       |                        /
 *  [180,190]              [130,150]
 *       |
 *       |      /
 *            /
 *           /
 *         |_
 *       x3
 *
 * \see http://www.ics.uci.edu/~csp/R40.pdf p.18
 */
BOOST_AUTO_TEST_CASE(example_network_0)
{
    TemporalConstraintNetwork tcn,expected;
    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v0->setLabel("v0");
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v1->setLabel("v1");
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v2->setLabel("v2");
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v3->setLabel("v3");

    // Designing start
    {
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
    }

    // Designing expected
    {
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
        IntervalConstraint::Ptr e5(new IntervalConstraint(v0,v2));
        e5->addInterval(Bounds(30,50));

        expected.addIntervalConstraint(e0);
        expected.addIntervalConstraint(e1);
        expected.addIntervalConstraint(e2);
        expected.addIntervalConstraint(e3);
        expected.addIntervalConstraint(e4);
        expected.addIntervalConstraint(e5);
    }

    TemporalConstraintNetwork intersectionNetwork = LoosePathConsistency::intersectionNetwork(tcn);
    TemporalConstraintNetwork result = LoosePathConsistency::looseIntersectionNetwork(tcn, intersectionNetwork);

    BaseGraph::Ptr graph = result.getDistanceGraph();
    BOOST_TEST_MESSAGE("Write result graph: " << graph->size() << " order: " << graph->order());
    graph_analysis::io::GraphIO::write("/tmp/templ-test-loose_path_consistency-example_0-loose_intersection_network-result.gexf", graph);
    BOOST_REQUIRE_MESSAGE(expected.equals(graph), "Expected equal graphs, actual: "<<expected.equals(graph));
}

/**
 *       x0   -- [10,20] [100,110]   --> X1
 *       |                          /   |
 *       |                        /
 *  [80,100]          [30,40]   /
 *  [150,160]       [130,150*!] /          [20,40]
 *  [180,190]                            [100,130]
 *       |        /
 *       |      /
 *            /
 *           /
 *         |_
 *       x3 <--                      -- x2
 *             [50,70] [110,120]
 *             [130,140] [160,190]
 *
 * \see http://www.ics.uci.edu/~csp/R40.pdf p.18
 *
 * *!There is actually an error in the example the paper provides, it has to be
 * [30,40][130,140] for the intervall between x3->x1
 */
BOOST_AUTO_TEST_CASE(example_network_1)
{
    TemporalConstraintNetwork tcn,expected;
    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v0->setLabel("v0");
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v1->setLabel("v1");
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v2->setLabel("v2");
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v3->setLabel("v3");

    // Designing start
    {
        IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1));
        i0->addInterval(Bounds(10,20));
        i0->addInterval(Bounds(100,110));

        IntervalConstraint::Ptr i1(new IntervalConstraint(v1,v2));
        i1->addInterval(Bounds(20,40));
        i1->addInterval(Bounds(100, 130));

        IntervalConstraint::Ptr i2(new IntervalConstraint(v2,v3));
        i2->addInterval(Bounds(50, 70));
        i2->addInterval(Bounds(110, 120));
        i2->addInterval(Bounds(130, 140));
        i2->addInterval(Bounds(160, 190));

        IntervalConstraint::Ptr i3(new IntervalConstraint(v1,v3));
        i3->addInterval(Bounds(30,40));
        i3->addInterval(Bounds(130,140));

        IntervalConstraint::Ptr i4(new IntervalConstraint(v0,v3));
        i4->addInterval(Bounds(80,100));
        i4->addInterval(Bounds(150,160));
        i4->addInterval(Bounds(180,190));

        tcn.addIntervalConstraint(i0);
        tcn.addIntervalConstraint(i1);
        tcn.addIntervalConstraint(i2);
        tcn.addIntervalConstraint(i3);
        tcn.addIntervalConstraint(i4);
    }

    // Designing expected
    {
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
    }

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 5,"Expected: 5 edges, Actual: "<<tcn.getEdgeNumber());

    TemporalConstraintNetwork result = LoosePathConsistency::loosePathConsistency(tcn);

    BaseGraph::Ptr graph = result.getDistanceGraph();
    {
        BOOST_TEST_MESSAGE("Write result graph: " << graph->size() << " order: " << graph->order());
        EdgeIterator::Ptr e = graph->getEdgeIterator();
        while(e->next())
        {
            Edge::Ptr edge = e->current();
            BOOST_TEST_MESSAGE(edge->toString() << " " << edge->getLabel());
        }
        VertexIterator::Ptr v = graph->getVertexIterator();
        while(v->next())
        {
            Vertex::Ptr vertex = v->current();
            BOOST_TEST_MESSAGE(vertex->toString() << " " << vertex->getLabel());
        }
    }
    {
        BOOST_TEST_MESSAGE("Write expected graph: " << expected.getDistanceGraph()->size() << " order: " << expected.getDistanceGraph()->order());
        EdgeIterator::Ptr e = expected.getDistanceGraph()->getEdgeIterator();
        while(e->next())
        {
            Edge::Ptr edge = e->current();
            BOOST_TEST_MESSAGE(edge->toString() << " " << edge->getLabel());
        }
        VertexIterator::Ptr v = expected.getDistanceGraph()->getVertexIterator();
        while(v->next())
        {
            Vertex::Ptr vertex = v->current();
            BOOST_TEST_MESSAGE(vertex->toString() << " " << vertex->getLabel());
        }
    }

    graph_analysis::io::GraphIO::write("/tmp/templ-test-loose_path_consistency-example_1-network-result.gexf", graph);
    BOOST_REQUIRE_MESSAGE(expected.equals(graph), "Expected equal graphs, actual: "<<expected.equals(graph));
}


BOOST_AUTO_TEST_CASE(intersection_network)
{
    TemporalConstraintNetwork tcn,expected;
    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v0->setLabel("v0");
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v1->setLabel("v1");
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v2->setLabel("v2");
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v3->setLabel("v3");

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

    graph_analysis::io::GraphIO::write("/tmp/templ-test-loose_path_consistency-intersection_network-result.gexf", graph);

    BOOST_REQUIRE_MESSAGE(expected.equals(graph), "Expected: 1, Actual: "<<expected.equals(graph));
}

BOOST_AUTO_TEST_CASE(loose_intersection)
{
    TemporalConstraintNetwork tcn,tcn2,expected;

    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v0->setLabel("v0");
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v1->setLabel("v1");
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v2->setLabel("v2");
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    v3->setLabel("v3");

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

    TemporalConstraintNetwork result = LoosePathConsistency::looseIntersectionNetwork(tcn,tcn2);
    BaseGraph::Ptr graph = result.getDistanceGraph();

    graph_analysis::io::GraphIO::write("/tmp/templ-test-loose_path_consistency-loose_network-result.gexf", graph);
    BOOST_REQUIRE_MESSAGE(expected.equals(graph), "Expected: 1, Actual: "<<expected.equals(graph));

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
    .                [23,29]
  [1,2]           [1,22]
  [11,12]        .
  [21,22]     .
    .      .
    .   .
    v0

*/
BOOST_AUTO_TEST_CASE(lpc)
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
    i3->addInterval(Bounds(23,29));
    i3->addInterval(Bounds(34,46));

    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    tcn.addIntervalConstraint(i2);

    expected.addIntervalConstraint(i0);
    expected.addIntervalConstraint(i1);
    expected.addIntervalConstraint(i3);

    TemporalConstraintNetwork result = LoosePathConsistency::loosePathConsistency(tcn);
    BaseGraph::Ptr graph = result.getDistanceGraph();

    graph_analysis::io::GraphIO::write("/tmp/templ-test-loose_path_consistency-lpc-result.gexf", graph);
    BOOST_REQUIRE_MESSAGE(expected.equals(graph),"Expected: 1, Actual: "<<expected.equals(graph));
}

BOOST_AUTO_TEST_SUITE_END()
