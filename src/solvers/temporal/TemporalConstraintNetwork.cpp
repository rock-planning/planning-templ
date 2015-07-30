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

void TemporalConstraintNetwork::addInterval(TimePoint::Ptr source, TimePoint::Ptr target, const Bounds& bounds)
{
    // Upper and lower bound are added as edges in forward and backward
    // direction between two edges
    // A --- weight: upper bound   --> B
    // B --- weight: - lower bound --> A
    // the lower bound will be added as negative cost
    using namespace graph_analysis;
    {
        WeightedEdge::Ptr edge(new WeightedEdge(bounds.getUpperBound()));
        edge->setSourceVertex(source);
        edge->setTargetVertex(target);
        mpDistanceGraph->addEdge(edge);
    }

    {
        WeightedEdge::Ptr edge(new WeightedEdge(- bounds.getLowerBound()));
        edge->setSourceVertex(target);
        edge->setTargetVertex(source);
        mpDistanceGraph->addEdge(edge);
    }
}

/*graph_analysis::BaseGraph::Ptr*/int TemporalConstraintNetwork::stp()
{
	BaseGraph::Ptr graph = mpDistanceGraph->copy();
	VertexIterator::Ptr vit = graph->getVertexIterator();
	
	WeightedEdge::Ptr edge;
	TimePoint::Ptr current, next;
	
	int cnt=0;
	double max1=0, min1=0;
	while (vit->next())
	{
		current = boost::dynamic_pointer_cast<TimePoint>(vit->current());
		EdgeIterator::Ptr edgeIt = graph->getEdgeIterator(current);
		vit->next();
		next = boost::dynamic_pointer_cast<TimePoint>(vit->current());
		
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
	mpDistanceGraph = graph;
	return cnt;
}

/*graph_analysis::BaseGraph::Ptr*/void TemporalConstraintNetwork::intersection(graph_analysis::BaseGraph::Ptr a)
{
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
