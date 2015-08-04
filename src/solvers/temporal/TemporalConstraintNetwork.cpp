#include "TemporalConstraintNetwork.hpp"
//#include <numeric/Combinatorics.hpp>
//#include <base/Logging.hpp>
#include <graph_analysis/WeightedEdge.hpp>
//#include <graph_analysis/algorithms/FloydWarshall.hpp>

using namespace templ::solvers::temporal::point_algebra;
using namespace graph_analysis;


namespace templ {
namespace solvers {
namespace temporal {

TemporalConstraintNetwork::TemporalConstraintNetwork()
    : mpDistanceGraph( new graph_analysis::lemon::DirectedGraph() )
{
}

TemporalConstraintNetwork::~TemporalConstraintNetwork()
{
}

void TemporalConstraintNetwork::addTimePoint(point_algebra::TimePoint::Ptr t)
{
    mpDistanceGraph->addVertex(t);
}

void TemporalConstraintNetwork::addIntervalConstraint(IntervalConstraint::Ptr i)
{
	using namespace graph_analysis;
/*	
	WeightedEdge::Ptr edge(new WeightedEdge(bounds.getUpperBound()));
    edge->setSourceVertex(source);
    edge->setTargetVertex(target);
*/
    std::stringstream ss;
    ss << "Interval: [" << i->getLowerBound() << "," << i->getUpperBound() << "]"; 
    Edge::Ptr edge(new Edge(ss.str()));
    edge->setSourceVertex(i->getSourceVertex());
    edge->setTargetVertex(i->getTargetVertex());
	mpDistanceGraph->addEdge(edge);
}

/*graph_analysis::BaseGraph::Ptr*/int TemporalConstraintNetwork::stp()
{
	BaseGraph::Ptr graph = mpDistanceGraph->copy();
	VertexIterator::Ptr vit = graph->getVertexIterator();
	
	EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
	
	int cnt=0;
	Variable::Ptr current,next;
	IntervalConstraint::Ptr edge;
	double max1=0, min1=0;

	while (vit->next())
	{
		current = boost::dynamic_pointer_cast<Variable>(vit->current());
		EdgeIterator::Ptr edgeIt = graph->getEdgeIterator(current);

		vit->next();
		next = boost::dynamic_pointer_cast<Variable>(vit->current());
		max1 = 0;
		min1 = 10000;
		while (edgeIt->next())
		{
			edge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
			if (boost::dynamic_pointer_cast<Variable>(edge->getSourceVariable) == next || boost::dynamic_pointer_cast<Variable>(edge->getTargetVariable) == next)
			{
				cnt++;
			}
		} 
	}
/*	
	while (vit->next())
	{
		
		current = boost::dynamic_pointer_cast<Variable>(vit->current());
		EdgeIterator::Ptr edgeIt = graph->getEdgeIterator(current);
		
		vit->next();

		next = boost::dynamic_pointer_cast<Variable>(vit->current());
		
		max1= 0;
		min1= 0;

		while (edgeIt->next())
		{
			//e = edgeIt->current();
			edge = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt->current() );
			if ((boost::dynamic_pointer_cast<TimePoint>(edge->getSourceVertex()) == next || boost::dynamic_pointer_cast<TimePoint>(edge->getTargetVertex()) == next))
				if (max1 < edge->getWeight()) max1 = edge->getWeight();
				else
				if (min1 > edge->getWeight()) min1 = edge->getWeight();
		}
		//if (cnt!= 0) return min1;
		EdgeIterator::Ptr edgeIt2 = graph->getEdgeIterator(current);
		while (edgeIt2->next())
		{
			//e = edgeIt->current();
			edge = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt2->current() );
			if ((boost::dynamic_pointer_cast<TimePoint>(edge->getSourceVertex()) == next || boost::dynamic_pointer_cast<TimePoint>(edge->getTargetVertex()) == next) && max1 != edge->getWeight() && min1 != edge->getWeight())
			{
				graph->removeEdge(edge);
				cnt++;
			}
		}		
	}
*/
	mpDistanceGraph = graph->copy();
	return cnt;
}

/*graph_analysis::BaseGraph::Ptr*/double TemporalConstraintNetwork::intersection(TemporalConstraintNetwork other)
{
	//graph1 is the simple temporal graph
	BaseGraph::Ptr graph0 = mpDistanceGraph->copy();
	BaseGraph::Ptr graph1 = mpDistanceGraph->copy();
	BaseGraph::Ptr graph2 = other.mpDistanceGraph->copy();
	BaseGraph::Ptr graph3 = other.mpDistanceGraph->copy();
	//final graph
	BaseGraph::Ptr graph;
	BaseGraph::Ptr graph4 = mpDistanceGraph->copy();
	VertexIterator::Ptr vit = graph4->getVertexIterator();

	while (vit->next()) 
	{
		Vertex::Ptr current = vit->current();
		//graph->addVertex(current);
	}
	EdgeIterator::Ptr edgeIt0 = graph0->getEdgeIterator();
	EdgeIterator::Ptr edgeIt1 = graph1->getEdgeIterator();
	EdgeIterator::Ptr edgeIt2 = graph2->getEdgeIterator();
	EdgeIterator::Ptr edgeIt3 = graph3->getEdgeIterator();
	//while (edgeIt->next())
	//{

	//}
/*
	edgeIt1->next();
	edgeIt2->next();
	WeightedEdge::Ptr edge1 = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt1->current() );
	WeightedEdge::Ptr edge2 = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt2->current() );
*/	double cnt=0;
/*	while (edgeIt0->next())
	{
		// move the pointer to the first negative weight
		while (boost::dynamic_pointer_cast<WeightedEdge>(edgeIt1->current())->getWeight() > 0) edgeIt1->next();
		
		while (edgeIt2->next())
		{
			// move the pointer to the first negative weight
			while (boost::dynamic_pointer_cast<WeightedEdge>(edgeIt3->current())->getWeight() > 0) edgeIt3->next();

			//now compare if the absolute value of the negative weight (from graph3) is bigger than the positive weight (from graph0) 
			//and if the positive value of the negative weight (from graph1) is less than the positive weight (from graph2)
			//if one of them satisfy, then we have to remove that constraint 
			WeightedEdge::Ptr edge0 = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt0->current() );
			WeightedEdge::Ptr edge1 = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt1->current() );
			WeightedEdge::Ptr edge2 = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt2->current() );
			WeightedEdge::Ptr edge3 = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt3->current() );

			if (edge0->getWeight() > edge2->getWeight()) 
			{
				if (edge1->getWeight() > edge3->getWeight()) 
					{
						WeightedEdge::Ptr newEdge0(new WeightedEdge(edge2->getWeight()));
						newEdge0->setSourceVertex(edge0->getSourceVertex());
						newEdge0->setTargetVertex(edge0->getTargetVertex());
						graph->addEdge(newEdge0);

						WeightedEdge::Ptr newEdge1(new WeightedEdge(edge3->getWeight()));
						newEdge1->setSourceVertex(edge0->getTargetVertex());
						newEdge1->setTargetVertex(edge0->getSourceVertex());
						graph->addEdge(newEdge1);

						cnt++;
					}
					else
					{
						WeightedEdge::Ptr newEdge0(new WeightedEdge(edge2->getWeight()));
						newEdge0->setSourceVertex(edge0->getSourceVertex());
						newEdge0->setTargetVertex(edge0->getTargetVertex());
						graph->addEdge(newEdge0);

						WeightedEdge::Ptr newEdge1(new WeightedEdge(edge1->getWeight()));
						newEdge1->setSourceVertex(edge0->getTargetVertex());
						newEdge1->setTargetVertex(edge0->getSourceVertex());
						graph->addEdge(newEdge1);

						cnt++;
					}
			}
			else
			{
				if (edge1->getWeight() > edge3->getWeight()) 
					{
						WeightedEdge::Ptr newEdge0(new WeightedEdge(edge0->getWeight()));
						newEdge0->setSourceVertex(edge0->getSourceVertex());
						newEdge0->setTargetVertex(edge0->getTargetVertex());
						graph->addEdge(newEdge0);

						WeightedEdge::Ptr newEdge1(new WeightedEdge(edge3->getWeight()));
						newEdge1->setSourceVertex(edge0->getTargetVertex());
						newEdge1->setTargetVertex(edge0->getSourceVertex());
						graph->addEdge(newEdge1);

						cnt++;	
					}
					else
					{
						WeightedEdge::Ptr newEdge0(new WeightedEdge(edge0->getWeight()));
						newEdge0->setSourceVertex(edge0->getSourceVertex());
						newEdge0->setTargetVertex(edge0->getTargetVertex());
						graph->addEdge(newEdge0);

						WeightedEdge::Ptr newEdge1(new WeightedEdge(edge1->getWeight()));
						newEdge1->setSourceVertex(edge0->getTargetVertex());
						newEdge1->setTargetVertex(edge0->getSourceVertex());
						graph->addEdge(newEdge1);

						cnt++;
					}	
			}
		}
	}*/
	return cnt;
}

/*graph_analysis::BaseGraph::Ptr*/void TemporalConstraintNetwork::ult()
{
}

int TemporalConstraintNetwork::getEdgeNumber()
{
	EdgeIterator::Ptr edgeIt = mpDistanceGraph->getEdgeIterator();
	int cnt=0;
	while (edgeIt->next()) cnt++;
	return cnt;
}
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
