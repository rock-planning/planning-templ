#include "TemporalConstraintNetwork.hpp"
//#include <numeric/Combinatorics.hpp>
//#include <base/Logging.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/algorithms/FloydWarshall.hpp>

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
	mpDistanceGraph->addEdge(i);
}

void TemporalConstraintNetwork::stp()
{
	BaseGraph::Ptr graph = mpDistanceGraph->copy();
	
	TemporalConstraintNetwork tcn;
	
	double n = tcn.getEdgeNumber();
	EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
	
	IntervalConstraint::Ptr edge,prevEdge;
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
					prevEdge = edge;
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
				if (max < edge->getUpperBound()) max = edge->getUpperBound();
				if (min > edge->getLowerBound()) min = edge->getLowerBound();
				
				IntervalConstraint::Ptr i(new IntervalConstraint(source0,target0,min,max));
				tcn.addIntervalConstraint(i);
			}
			else
			{
				IntervalConstraint::Ptr i(new IntervalConstraint(edge->getSourceVariable(),edge->getTargetVariable(),edge->getLowerBound(),edge->getUpperBound()));
				tcn.addIntervalConstraint(i);
			}
			prevEdge = edge;
			edge = nextEdge;
		}
		else
		{
			if (n!=1)
			{
				if (prevEdge->getSourceVariable() != edge->getSourceVariable() || prevEdge->getTargetVariable() != edge->getTargetVariable())
				{
					IntervalConstraint::Ptr i(new IntervalConstraint(edge->getSourceVariable(),edge->getTargetVariable(),edge->getLowerBound(),edge->getUpperBound()));
					tcn.addIntervalConstraint(i);
				}
			}
			else
			{
				IntervalConstraint::Ptr i(new IntervalConstraint(edge->getSourceVariable(),edge->getTargetVariable(),edge->getLowerBound(),edge->getUpperBound()));
				tcn.addIntervalConstraint(i);
			}
			break;
		}
	}
	mpDistanceGraph = tcn.mpDistanceGraph->copy();
}

graph_analysis::BaseGraph::Ptr TemporalConstraintNetwork::intersection(graph_analysis::BaseGraph::Ptr other)
{
	//graph0 is the simple temporal network graph
	BaseGraph::Ptr graph0 = mpDistanceGraph->copy();
	BaseGraph::Ptr graph1 = other->copy();
	//final graph
	TemporalConstraintNetwork tcn;

	EdgeIterator::Ptr edgeIt1 = graph1->getEdgeIterator();
	Variable::Ptr source,target;
	IntervalConstraint::Ptr edge0,edge1;

	double cnt=0;
	
	while (edgeIt1->next()) 
	{
		edge1 = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt1->current() );
		source = edge1 -> getSourceVariable();
		target = edge1 -> getTargetVariable();
		EdgeIterator::Ptr edgeIt0 = graph0->getEdgeIterator();
		while (edgeIt0->next())
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
				else
				if (edge1->getLowerBound() <= edge0->getLowerBound() && edge1->getUpperBound() <= edge0->getUpperBound())
				{
					if (edge0->getLowerBound() <= edge1->getUpperBound())
					{
						IntervalConstraint::Ptr i(new IntervalConstraint(source, target, edge0->getLowerBound(), edge1->getUpperBound()));
						tcn.addIntervalConstraint(i);
						cnt++;
					}
				}
				else
				if (edge1->getLowerBound() >= edge0->getLowerBound() && edge1->getUpperBound() >= edge0->getUpperBound())
				{
					if (edge0->getUpperBound() >= edge1->getLowerBound())
					{
						IntervalConstraint::Ptr i(new IntervalConstraint(source, target, edge1->getLowerBound(), edge0->getUpperBound()));
						tcn.addIntervalConstraint(i);
						cnt++;
					}
				}
				else
				if (edge1->getLowerBound() >= edge0->getLowerBound() && edge1->getUpperBound() <= edge0->getUpperBound())
				{
					IntervalConstraint::Ptr i(new IntervalConstraint(source, target, edge1->getLowerBound(), edge1->getUpperBound()));
					tcn.addIntervalConstraint(i);
					cnt++;
				}
			}
		}
	}
	mpDistanceGraph = tcn.mpDistanceGraph->copy();
	return mpDistanceGraph;
}


graph_analysis::BaseGraph::Ptr TemporalConstraintNetwork::toWeightedGraph()
{
	BaseGraph::Ptr graph(new graph_analysis::lemon::DirectedGraph());
	BaseGraph::Ptr tcn = mpDistanceGraph->copy();
	EdgeIterator::Ptr edgeIt = tcn->getEdgeIterator();
	//VertexIterator::Ptr vit = tcn->getVertexIterator();

	//while (vit->next()) graph->addVertex(vit->current);

	while (edgeIt->next())
    {
    	IntervalConstraint::Ptr edge0 = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
        WeightedEdge::Ptr edge1(new WeightedEdge(edge0->getUpperBound()));
        edge1->setSourceVertex(edge0->getSourceVariable());
        edge1->setTargetVertex(edge0->getTargetVariable());
        graph->addEdge(edge1);

        WeightedEdge::Ptr edge2(new WeightedEdge(-edge0->getLowerBound()));
        edge2->setSourceVertex(edge0->getTargetVariable());
        edge2->setTargetVertex(edge0->getSourceVariable());
        graph->addEdge(edge2);
    }

    return graph;
}


void TemporalConstraintNetwork::minNetwork()
{
	TemporalConstraintNetwork tcn;
	BaseGraph::Ptr graph = (toWeightedGraph())->copy();
	BaseGraph::Ptr oldGraph = mpDistanceGraph->copy();
	algorithms::DistanceMatrix distanceMatrix = algorithms::FloydWarshall::allShortestPaths(graph, [](Edge::Ptr e) -> double
                {
                    return boost::dynamic_pointer_cast<WeightedEdge>(e)->getWeight();
                });
	EdgeIterator::Ptr edgeIt = oldGraph->getEdgeIterator();
	//vit->next();
	double cnt=0;
	Variable::Ptr v1,v2;
	while (edgeIt->next())
	{
		IntervalConstraint::Ptr edge = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
		v1 = edge->getSourceVariable();
		v2 = edge->getTargetVariable();
	
		cnt++; 

		double distance12 = distanceMatrix[std::pair<Variable::Ptr, Variable::Ptr>(v1,v2)];
		double distance21 = distanceMatrix[std::pair<Variable::Ptr, Variable::Ptr>(v2,v1)];
		distance21 = (-1)*distance21;
		IntervalConstraint::Ptr i(new IntervalConstraint(v1, v2, distance21, distance12));
		tcn.addIntervalConstraint(i);	
	}
	mpDistanceGraph = tcn.mpDistanceGraph->copy();
}

bool TemporalConstraintNetwork::areEqual(graph_analysis::BaseGraph::Ptr other)
{
	EdgeIterator::Ptr edgeIt1 = mpDistanceGraph -> getEdgeIterator();
	EdgeIterator::Ptr edgeIt2 = other -> getEdgeIterator();
	while (edgeIt1->next() && edgeIt2->next())
	{
		IntervalConstraint::Ptr i1 = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt1->current());
		IntervalConstraint::Ptr i2 = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt2->current());
		if (i1->getSourceVariable() != i2->getSourceVariable()) return false;
		if (i1->getTargetVariable() != i2->getTargetVariable()) return false;
		if (i1->getLowerBound() != i2->getLowerBound()) return false;
		if (i1->getUpperBound() != i2->getUpperBound()) return false;
	}
	return true;
}

void TemporalConstraintNetwork::upperLowerTightening()
{
	int cnt=0;
	BaseGraph::Ptr oldGraph = mpDistanceGraph->copy();
	do
	{
		cnt++;
		oldGraph = mpDistanceGraph->copy();
		stp();
		minNetwork();
		EdgeIterator::Ptr edgeIt = oldGraph->getEdgeIterator();
		TemporalConstraintNetwork tcn;
		while (edgeIt->next())
		{
			IntervalConstraint::Ptr i = boost::dynamic_pointer_cast<IntervalConstraint>(edgeIt->current());
			IntervalConstraint::Ptr n(new IntervalConstraint(i->getSourceVariable(),i->getTargetVariable(),i->getLowerBound(),i->getUpperBound()));
			tcn.addIntervalConstraint(n);
		}
		tcn.intersection(mpDistanceGraph);
		mpDistanceGraph = (tcn.mpDistanceGraph) -> copy();
		oldGraph = oldGraph ->copy();
	} while (!areEqual(oldGraph));
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
