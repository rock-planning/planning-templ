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

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1));
    i0->addInterval(Bounds(10,40));
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

    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    tcn.addIntervalConstraint(i2);
    tcn.addIntervalConstraint(i3);
    tcn.addIntervalConstraint(i4);
    

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 5,"Expected: 5, Actual: "<<tcn.getEdgeNumber());

    tcn.upperLowerTightening();

    BaseGraph::Ptr graph = tcn.getDistanceGraph();
    EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
    while (edgeIt->next())
    {
        IntervalConstraint::Ptr i = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
        if (i->getSourceVariable() == v0 && i->getTargetVariable() == v1)
        {
            BOOST_REQUIRE_MESSAGE(i->getIntervalsNumber() == 1,"Expected: 1, Actual: "<<i->getIntervalsNumber());
        }
        if (i->getSourceVariable() == v1 && i->getTargetVariable() == v2)
        {
            BOOST_REQUIRE_MESSAGE(i->getIntervalsNumber() == 2,"Expected: 2, Actual: "<<i->getIntervalsNumber());
        }
        if (i->getSourceVariable() == v2 && i->getTargetVariable() == v3)
        {
            BOOST_REQUIRE_MESSAGE(i->getIntervalsNumber() == 1,"Expected: 1, Actual: "<<i->getIntervalsNumber());
        }
        if (i->getSourceVariable() == v1 && i->getTargetVariable() == v3)
        {
            BOOST_REQUIRE_MESSAGE(i->getIntervalsNumber() == 2,"Expected: 2, Actual: "<<i->getIntervalsNumber());
        }
        if (i->getSourceVariable() == v0 && i->getTargetVariable() == v3)
        {
            BOOST_REQUIRE_MESSAGE(i->getIntervalsNumber() == 1,"Expected: 1, Actual: "<<i->getIntervalsNumber());
        }
    }    
}

BOOST_AUTO_TEST_SUITE_END()