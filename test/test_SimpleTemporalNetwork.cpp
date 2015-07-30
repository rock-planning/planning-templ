#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/SimpleTemporalNetwork.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;

BOOST_AUTO_TEST_SUITE(simple_temporal_network)

BOOST_AUTO_TEST_CASE(value_propagation_1)
{
    {
        // [10,80] -- 38 --> [30,100]
        //    <------ -30 ----|
        // Expected after propagation
        // [10,70] -- 38 --> [48,100]
        SimpleTemporalNetwork stn;
        point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,80));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(30,100));
        //point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(0,100));
      
        stn.addInterval(tp0, tp1, Bounds(30,38));
        // Using for loop to check stability of result
        for(int i = 0; i < 2; ++i)
        {
            graph_analysis::BaseGraph::Ptr baseGraph = stn.propagate();

            BOOST_REQUIRE_MESSAGE( tp0->getUpperBound() == 70, "Upper bound corrected after propagation: expected 70, actual " << tp0->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp1->getLowerBound() == 48, "Lower bound corrected after propagation: expected 48, actual " << tp1->getLowerBound());

        }


        // [10,80] -- 38 --> [30,100] -- 30 --> [70,100]
        //    <------ -30 ----|    <--- -10 ------|
        // Expected after propagation
        // [10,60] -- 38 --> [48,90] --- 30 --> [78,100]
        //    <------ -30 ----|    <--- -10 ------|
        point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(70,100));
        stn.addInterval(tp1,tp2, Bounds(10,30));
        {
            graph_analysis::BaseGraph::Ptr baseGraph = stn.getDistanceGraph();
            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-before-domain_propagation", baseGraph, graph_analysis::representation::GEXF);
            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-before-domain_propagation", baseGraph, graph_analysis::representation::GRAPHVIZ);
        }
        // Using for loop to check stability of result
        for(int i = 0; i < 2; ++i)
        {
            stn.propagate();
            graph_analysis::BaseGraph::Ptr baseGraph = stn.getDistanceGraph();
            BOOST_REQUIRE_MESSAGE( tp0->getUpperBound() == 60, "Upper bound corrected after propagation: expected 60, actual " << tp0->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp1->getLowerBound() == 48, "Lower bound corrected after propagation: expected 48, actual " << tp1->getLowerBound());
            BOOST_REQUIRE_MESSAGE( tp1->getUpperBound() == 90, "Upper bound corrected after propagation: expected 90, actual " << tp1->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp2->getLowerBound() == 78, "Lower bound corrected after propagation: expected 78, actual " << tp2->getLowerBound());

            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-after-domain_propagation", baseGraph, graph_analysis::representation::GEXF);
            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-after-domain_propagation", baseGraph, graph_analysis::representation::GRAPHVIZ);
        }
    }
}

BOOST_AUTO_TEST_CASE(value_propagation_2)
{
    {
        SimpleTemporalNetwork stn;

        point_algebra::TimePoint::Ptr tp0(new point_algebra::TimePoint(10,20));
        point_algebra::TimePoint::Ptr tp1(new point_algebra::TimePoint(0,1000));
        point_algebra::TimePoint::Ptr tp2(new point_algebra::TimePoint(0,1000));
        point_algebra::TimePoint::Ptr tp3(new point_algebra::TimePoint(60,70));
        //point_algebra::TimePoint::Ptr tp4(new point_algebra::TimePoint(0,0));
        
        stn.addInterval(tp0, tp1, Bounds(30,40));
        stn.addInterval(tp2, tp1, Bounds(10,20));
        stn.addInterval(tp2, tp3, Bounds(40,50));

        {
            graph_analysis::BaseGraph::Ptr baseGraph = stn.getDistanceGraph();
            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-before-domain_propagation", baseGraph, graph_analysis::representation::GEXF);
            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-before-domain_propagation", baseGraph, graph_analysis::representation::GRAPHVIZ);
        }

        // Using for loop to check stability of result
        for(int i = 0; i < 2; ++i)
        {
            stn.propagate();
            graph_analysis::BaseGraph::Ptr baseGraph = stn.getDistanceGraph();

            BOOST_REQUIRE_MESSAGE( tp0->getLowerBound() == 10, "Lower bound corrected after propagation: expected 10, actual " << tp0->getLowerBound());
            BOOST_REQUIRE_MESSAGE( tp0->getUpperBound() == 20, "Upper bound corrected after propagation: expected 20, actual " << tp0->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp1->getLowerBound() == 50, "Lower bound corrected after propagation: expected 50, actual " << tp1->getLowerBound());
            BOOST_REQUIRE_MESSAGE( tp1->getUpperBound() == 1000, "Upper bound corrected after propagation: expected 1000, actual " << tp1->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp2->getLowerBound() == 0, "Lower bound corrected after propagation: expected 0, actual " << tp2->getLowerBound());
            BOOST_REQUIRE_MESSAGE( tp2->getUpperBound() == 30, "Upper bound corrected after propagation: expected 30, actual " << tp2->getUpperBound());
            BOOST_REQUIRE_MESSAGE( tp3->getLowerBound() == 60, "Lower bound corrected after propagation: expected 60, actual " << tp3->getLowerBound());
            BOOST_REQUIRE_MESSAGE( tp3->getUpperBound() == 70, "Upper bound corrected after propagation: expected 70, actual " << tp3->getUpperBound());

            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-after-domain_propagation", baseGraph, graph_analysis::representation::GEXF);
            graph_analysis::io::GraphIO::write("test_SimpleTemporalNetwork-after-domain_propagation", baseGraph, graph_analysis::representation::GRAPHVIZ);

        }
    }
}
BOOST_AUTO_TEST_SUITE_END()
