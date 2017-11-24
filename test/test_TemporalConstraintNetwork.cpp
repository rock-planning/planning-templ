#include <boost/test/unit_test.hpp>
#include "test_utils.hpp"

#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/VertexTypeManager.hpp>
#include <graph_analysis/EdgeTypeManager.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;
using namespace graph_analysis;

BOOST_AUTO_TEST_SUITE(temporal_constraint_network)


BOOST_AUTO_TEST_CASE(ult)
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

    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::max()));
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::max()));
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::max()));
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::max()));

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
    bool ok = expected.equals(graph);
    BOOST_REQUIRE_MESSAGE(ok, "Expected: 1, Actual: " << ok);
}

BOOST_AUTO_TEST_CASE(io)
{
    using namespace templ::solvers::temporal;
    using namespace graph_analysis;

    QualitativeTemporalConstraintNetwork::Ptr tpc(new QualitativeTemporalConstraintNetwork());

    Vertex::Ptr t0 = VertexTypeManager::getInstance()->createVertex("QualitativeTimePoint", "t0");
    Vertex::Ptr t1 = VertexTypeManager::getInstance()->createVertex("QualitativeTimePoint", "t1");
    BOOST_REQUIRE_MESSAGE(t0->getClassName() == "QualitativeTimePoint", "QualitativeTimePoint is instanciated for t0");
    BOOST_REQUIRE_MESSAGE(t1->getClassName() == "QualitativeTimePoint", "QualitativeTimePoint is instanciated for t1");

    Edge::Ptr e0 = EdgeTypeManager::getInstance()->createEdge("QualitativeTimePointConstraint", t0,t1,"<");
    BOOST_REQUIRE_MESSAGE(e0->getClassName() == "QualitativeTimePointConstraint", "QualitativeTimePointConstraint is instanciated for e0");
}

BOOST_AUTO_TEST_CASE(io_non_overlapping_intervals)
{
    std::string filename = getRootDir() + "/test/data/temporal_constraint_networks/sequential-non-overlapping-intervals.gexf";

    using namespace templ::solvers::temporal;
    using namespace graph_analysis;

    QualitativeTemporalConstraintNetwork::Ptr tpc(new QualitativeTemporalConstraintNetwork());

    std::string dotFilename = "/tmp/test-sequential-non-overlapping-intervals.dot";
    graph_analysis::io::GraphIO::read(filename, tpc->getGraph());
    graph_analysis::io::GraphIO::write(dotFilename, tpc->getGraph());
    BOOST_TEST_MESSAGE("Dot file written to: " << dotFilename);

    std::map<std::string, point_algebra::TimePoint::Ptr> timepoints;

    VertexIterator::Ptr vertexIt = tpc->getGraph()->getVertexIterator();
    while(vertexIt->next())
    {
        point_algebra::TimePoint::Ptr tp = dynamic_pointer_cast<point_algebra::TimePoint>(vertexIt->current());
        BOOST_REQUIRE_MESSAGE(tp, "Timepoint: " << tp->toString() << " found");
        timepoints[tp->getLabel()] = tp;
    }

    point_algebra::TimePointComparator comparator(tpc);
    BOOST_REQUIRE_MESSAGE( comparator.greaterThan(timepoints["t9"], timepoints["t0"]), "t9 > t0");
    BOOST_REQUIRE_MESSAGE( !comparator.greaterThan(timepoints["t6"], timepoints["t9"]), "not t6 > t9");
    BOOST_REQUIRE_MESSAGE( !comparator.greaterThan(timepoints["t0"], timepoints["t9"]), "not t0 > t9");
    BOOST_REQUIRE_MESSAGE( !comparator.lessThan(timepoints["t9"], timepoints["t0"]), "not t9 < t0");
}

BOOST_AUTO_TEST_SUITE_END()
