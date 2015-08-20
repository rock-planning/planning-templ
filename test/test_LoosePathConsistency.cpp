#include <boost/test/unit_test.hpp>
#include <templ/solvers/temporal/LoosePathConsistency.hpp>

using namespace templ::solvers;
using namespace templ::solvers::temporal;
using namespace graph_analysis;

BOOST_AUTO_TEST_SUITE(loose_path_consistency)


BOOST_AUTO_TEST_CASE(test_lpc)
{
	std::vector<Bounds> a;
	a.push_back(Bounds(80,100));
	a.push_back(Bounds(150,160));
	a.push_back(Bounds(180,190));
	std::vector<Bounds> b;
	b.push_back(Bounds(40,60));
	b.push_back(Bounds(130,160));
	b.push_back(Bounds(230,250));

	std::vector<Bounds> result = LoosePathConsistency::looseIntersection(a,b);
	std::vector<Bounds>::iterator it = result.begin();
	int cnt=0;
	while (it!=result.end())
	{
		cnt++;
		it++;
	}
	BOOST_REQUIRE_MESSAGE(cnt==1,"Expected: 1, Actual: "<<cnt);
	it = result.begin();
	BOOST_REQUIRE_MESSAGE(it->getLowerBound() == 150,"Expected: 150, Actual: " << it->getLowerBound());
	BOOST_REQUIRE_MESSAGE(it->getUpperBound() == 160,"Expected: 160, Actual: " << it->getUpperBound());

/*
	result = LoosePathConsistency::composition(a,b);
	cnt = 0;
	it = result.begin();
	while (it!=result.end())
	{
		cnt++;
		it++;
	}
	BOOST_REQUIRE_MESSAGE(cnt==8,"Expected: 8, Actual: "<<cnt);
*/
}

BOOST_AUTO_TEST_CASE(test_lpc_2)
{
	TemporalConstraintNetwork tcn;//,expected;

    point_algebra::TimePoint::Ptr v0(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v1(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v2(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));
    point_algebra::TimePoint::Ptr v3(new point_algebra::TimePoint(0,std::numeric_limits<double>::infinity()));

    IntervalConstraint::Ptr i0(new IntervalConstraint(v0,v1));
    i0->addInterval(Bounds(10,20));
    i0->addInterval(Bounds(110,110));
    IntervalConstraint::Ptr i1(new IntervalConstraint(v1,v2));
    i1->addInterval(Bounds(20,30));
    //i1->addInterval(Bounds(100,130));
    IntervalConstraint::Ptr i2(new IntervalConstraint(v2,v3));
    //i2->addInterval(Bounds(50,70));
    i2->addInterval(Bounds(110,120));
    //i2->addInterval(Bounds(130,140));
    //i2->addInterval(Bounds(160,190));
    IntervalConstraint::Ptr i3(new IntervalConstraint(v1,v3));
    //i3->addInterval(Bounds(30,40));
    i3->addInterval(Bounds(130,140));
    IntervalConstraint::Ptr i4(new IntervalConstraint(v0,v3));
    //i4->addInterval(Bounds(80,100));
    i4->addInterval(Bounds(150,160));
    //i4->addInterval(Bounds(180,190));
    IntervalConstraint::Ptr i5(new IntervalConstraint(v0,v2));
    i5->addInterval(Bounds(30,140));
    //i5->addInterval(Bounds(120,140));

    tcn.addIntervalConstraint(i0);
    tcn.addIntervalConstraint(i1);
    tcn.addIntervalConstraint(i2);
    tcn.addIntervalConstraint(i3);
    tcn.addIntervalConstraint(i4);
    tcn.addIntervalConstraint(i5);

    BOOST_REQUIRE_MESSAGE(tcn.getEdgeNumber() == 6,"Expected: 6, Actual: "<<tcn.getEdgeNumber());

    TemporalConstraintNetwork aux = LoosePathConsistency::intersectionNetwork(tcn);
    TemporalConstraintNetwork expected = LoosePathConsistency::looseNetwork(aux,tcn);
    BaseGraph::Ptr graph = expected.getDistanceGraph();   
    EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
    edgeIt->next();
    IntervalConstraint::Ptr edge = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
    std::vector<Bounds> v = edge->getIntervals();
    int x0 = v.size();
std::stringstream ss;
std::vector<Bounds>::iterator it = v.begin();
    while (it!=v.end())
    {
    	ss << it->getLowerBound() << " " << it->getUpperBound() << " ";
    	it++;
    }
//BOOST_REQUIRE_MESSAGE(true == false, ss.str());

    edgeIt->next();
    edge = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
    v = edge->getIntervals();
    int x1 = v.size();

 //   std::stringstream ss;
 //   if (v.size()==2)
 //   {
   /* std::vector<Bounds>::iterator*/ it = v.begin();
    while (it!=v.end())
    {
    	ss << it->getLowerBound() << " " << it->getUpperBound() << " ";
    	it++;
    }
//}
//BOOST_REQUIRE_MESSAGE(true == false, ss.str());
    edgeIt->next();
    edge = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
    v = edge->getIntervals();
    int x2 = v.size();
//   if (v.size()==2)
//    {
  /*  std::vector<Bounds>::iterator*/ it = v.begin();
    while (it!=v.end())
    {
    	ss << it->getLowerBound() << " " << it->getUpperBound() << " ";
    	it++;
    }
//}
//BOOST_REQUIRE_MESSAGE(true == false, ss.str());
    edgeIt->next();
    edge = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
    v = edge->getIntervals();
    int x3 = v.size();
//   if (v.size()==2)
//    {
  /*  std::vector<Bounds>::iterator*/ it = v.begin();
    while (it!=v.end())
    {
    	ss << it->getLowerBound() << " " << it->getUpperBound() << " ";
    	it++;
    }
//}
//BOOST_REQUIRE_MESSAGE(true == false, ss.str());
    edgeIt->next();
    edge = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
    v = edge->getIntervals();
    int x4 = v.size();
 //  if (v.size()==2)
//    {
  /*  std::vector<Bounds>::iterator */it = v.begin();
    while (it!=v.end())
    {
    	ss << it->getLowerBound() << " " << it->getUpperBound() << " ";
    	it++;
    }
//}
//BOOST_REQUIRE_MESSAGE(true == false, ss.str());
    edgeIt->next();
    edge = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
    v = edge->getIntervals();
    int x5 = v.size();
   //if (v.size()==2)
    //{
  /*  std::vector<Bounds>::iterator*/ it = v.begin();
    while (it!=v.end())
    {
    	ss << it->getLowerBound() << " " << it->getUpperBound() << " ";
    	it++;
    }
//}
	//BOOST_REQUIRE_MESSAGE(true == false, ss.str());
    //BOOST_REQUIRE_MESSAGE(x0 == 0,"Expected: 0, Actual: "<<x0 << x1 << x2 << x3 << x4 << x5);

/*
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
*/
}

BOOST_AUTO_TEST_SUITE_END()