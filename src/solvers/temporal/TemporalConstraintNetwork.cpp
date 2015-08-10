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
	//using namespace graph_analysis;
	mpDistanceGraph->addEdge(i);
}

graph_analysis::BaseGraph::Ptr TemporalConstraintNetwork::stp()
{
	BaseGraph::Ptr graph = mpDistanceGraph->copy();
	//BaseGraph::Ptr graph2 = mpDistanceGraph->copy();
	TemporalConstraintNetwork tcn;
	//VertexIterator::Ptr vit = graph->getVertexIterator();
	
	EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
	
	double ok=0;
	//Variable::Ptr current,next;
	IntervalConstraint::Ptr edge;
	double max, min; 
	edgeIt->next();
	edge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
	while (true)
	{
		if (edgeIt->next())
		{
			IntervalConstraint::Ptr nextEdge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
			Variable::Ptr source0 = edge->getSourceVariable();
			Variable::Ptr source1 = nextEdge->getSourceVariable();
			Variable::Ptr target0 = edge->getTargetVariable();
			Variable::Ptr target1 = nextEdge->getTargetVariable();
			if (source0 == source1 && target0 == target1)
			{
				max = 0;
				min = std::numeric_limits<double>::infinity();
				while (source0 == source1 && target0 == target1)
				{
					if (max < edge->getUpperBound()) max = edge->getUpperBound();
					if (min > edge->getLowerBound()) min = edge->getLowerBound();
					edge = nextEdge;
					
					if (!(edgeIt->next())) 
					{
						break;
					}
					
					nextEdge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
					source0 = edge->getSourceVariable();
					source1 = nextEdge->getSourceVariable();
					target0 = edge->getTargetVariable();
					target1 = nextEdge->getTargetVariable();
					
				}

				IntervalConstraint::Ptr i(new IntervalConstraint(source0,target0,min,max));
				tcn.addIntervalConstraint(i);
				ok++;
			}
			else
			{
				IntervalConstraint::Ptr i(new IntervalConstraint(edge->getSourceVariable(),edge->getTargetVariable(),edge->getLowerBound(),edge->getUpperBound()));
				tcn.addIntervalConstraint(i);
				ok++;
			}
			edge = nextEdge;
		}
		else
		{
			break;
		}
	}

	mpDistanceGraph = tcn.mpDistanceGraph->copy();
	return mpDistanceGraph;
}

// other represents the simple temporal network
graph_analysis::BaseGraph::Ptr TemporalConstraintNetwork::intersection(TemporalConstraintNetwork other)
{
	//graph1 is the simple temporal graph
	BaseGraph::Ptr graph0 = mpDistanceGraph->copy();
	BaseGraph::Ptr graph1 = other.mpDistanceGraph->copy();
	//final graph
	TemporalConstraintNetwork tcn;

	EdgeIterator::Ptr edgeIt0 = graph0->getEdgeIterator();
	EdgeIterator::Ptr edgeIt1 = graph1->getEdgeIterator();
	Variable::Ptr source,target;
	IntervalConstraint::Ptr edge0,edge1;

	double cnt=0;
	edgeIt0->next();
	while (edgeIt1->next()) 
	{
		edge1 = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt1->current() );
		source = edge1 -> getSourceVariable();
		target = edge1 -> getTargetVariable();
		while (true)
		{
			edge0 = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt0->current() );
			if (target == edge0->getTargetVariable() && source == edge0->getSourceVariable())
			{
				if (edge1->getLowerBound() <= edge0->getLowerBound() && edge1->getUpperBound() >= edge0->getUpperBound())
				{
					IntervalConstraint::Ptr i(new IntervalConstraint(source, target, edge0->getLowerBound(), edge0->getUpperBound()));
					tcn.addIntervalConstraint(i);
					cnt++;
				}
					
				if (edge1->getLowerBound() <= edge0->getLowerBound() && edge1->getUpperBound() <= edge0->getUpperBound())
				{
					if (edge0->getLowerBound() <= edge1->getUpperBound())
					{
						IntervalConstraint::Ptr i(new IntervalConstraint(source, target, edge0->getLowerBound(), edge1->getUpperBound()));
						tcn.addIntervalConstraint(i);
						cnt++;
					}
				}

				if (edge1->getLowerBound() >= edge0->getLowerBound() && edge1->getUpperBound() >= edge0->getUpperBound())
				{
					if (edge0->getUpperBound() >= edge1->getLowerBound())
					{
						IntervalConstraint::Ptr i(new IntervalConstraint(source, target, edge1->getLowerBound(), edge0->getUpperBound()));
						tcn.addIntervalConstraint(i);
						cnt++;
					}
				}

				if (edge1->getLowerBound() >= edge0->getLowerBound() && edge1->getUpperBound() <= edge0->getUpperBound())
				{
					IntervalConstraint::Ptr i(new IntervalConstraint(source, target, edge1->getLowerBound(), edge1->getUpperBound()));
					tcn.addIntervalConstraint(i);
					cnt++;
				}
			}
			else
			{
				break;
			}

			if (!(edgeIt0->next())) break;
		}
	}
	mpDistanceGraph = tcn.mpDistanceGraph->copy();
	return mpDistanceGraph;
}

/*graph_analysis::BaseGraph::Ptr*/void TemporalConstraintNetwork::upperLowerTightening()
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
