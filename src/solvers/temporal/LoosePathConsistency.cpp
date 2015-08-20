#include "LoosePathConsistency.hpp"
#include <graph_analysis/BaseGraph.hpp> 

namespace templ {
namespace solvers {
namespace temporal {

std::vector<Bounds> LoosePathConsistency::intersection(std::vector<Bounds> a, std::vector<Bounds> b)
{
	std::vector<Bounds>::iterator ita = a.begin();
	std::vector<Bounds> result;
	while (ita!=a.end())
	{
		std::vector<Bounds>::iterator itb = b.begin();
		while (itb!=b.end())
		{
			/* edge1: [x--------------y]
		   	   edge0: 	 [a-------b]
			=> add:      [a-------b]
			*/    
			if (ita->getLowerBound() <= itb->getLowerBound() && ita->getUpperBound() >= itb->getUpperBound())
			{
				// if an interval is not already in the final set then we add it
				if (!checkInterval(result,Bounds(itb->getLowerBound(),itb->getUpperBound()))) result.push_back(Bounds(itb->getLowerBound(),itb->getUpperBound()));
			}
			else
			/*  edge1: [x------------y]
				edge0:      [a------------b]
				=> add:     [a-------y] 
			*/
			if (ita->getLowerBound() <= itb->getLowerBound() && ita->getUpperBound() <= itb->getUpperBound())
			{
				if (itb->getLowerBound() <= ita->getUpperBound())
				{
					if (!checkInterval(result,Bounds(itb->getLowerBound(),ita->getUpperBound()))) result.push_back(Bounds(itb->getLowerBound(),ita->getUpperBound()));
				}
			}
			else
			/*  edge1:        [x-----------y]
		    	edge0:    [a---------b]
		    	=> add:       [x-----b]
			*/
		    if (ita->getLowerBound() >= itb->getLowerBound() && ita->getUpperBound() >= itb->getUpperBound())
			{
				if (itb->getUpperBound() >= ita->getLowerBound())
				{
					if (!checkInterval(result,Bounds(ita->getLowerBound(),itb->getUpperBound()))) result.push_back(Bounds(ita->getLowerBound(),itb->getUpperBound()));
				}
			}
			else
			/*  edge1:     [x----y]
				edge0:  [a-----------b]
				=> add:    [x----y]
			*/
			if (ita->getLowerBound() >= itb->getLowerBound() && ita->getUpperBound() <= itb->getUpperBound())
			{
				if (!checkInterval(result,Bounds(ita->getLowerBound(),ita->getUpperBound()))) result.push_back(Bounds(ita->getLowerBound(),ita->getUpperBound()));
			}
			// otherwise:
			/*  edge1:  [x----y]
				edge0:            [a----b]
				or
				edge1:            [x----y]
				edge0:  [a----b]
				=> we don't add anything
			*/
			itb++;
		}
		ita++;
	}
	return result;
}

bool LoosePathConsistency::checkInterval(std::vector<Bounds> a, Bounds x)
{
	std::vector<Bounds>::iterator it = a.begin();
	while (it!=a.end())
	{
		if (it->getLowerBound() == x.getLowerBound() && it->getUpperBound() == x.getUpperBound()) return true;
		it++;
	}
	return false;
}

std::vector<Bounds> LoosePathConsistency::composition(std::vector<Bounds> a,std::vector<Bounds> b)
{
	std::vector<Bounds> result;
	std::vector<Bounds>::iterator ita = a.begin();
	while (ita!=a.end())
	{
		std::vector<Bounds>::iterator itb = b.begin();
		while (itb!=b.end())
		{
			Bounds x(ita->getLowerBound()+itb->getLowerBound(), ita->getUpperBound()+itb->getUpperBound());
			if (!checkInterval(result,x)) result.push_back(x);
			itb++;
		}
		ita++;
	}
	return result;
}

std::vector<Bounds> LoosePathConsistency::looseIntersection(std::vector<Bounds> a, std::vector<Bounds> b)
{
	std::vector<Bounds> result;
	std::vector<Bounds>::iterator ita = a.begin();
	while (ita!=a.end())
	{
		std::vector<Bounds> aux;
		aux.push_back(*ita);
		std::vector<Bounds> help = intersection(aux,b);
		std::vector<Bounds>::iterator ith = help.begin();
		double min,max;
		min = std::numeric_limits<double>::infinity();
		max = - std::numeric_limits<double>::infinity();
		while (ith!=help.end())
		{
			if (min > ith->getLowerBound()) min = ith->getLowerBound();
			if (max < ith->getUpperBound()) max = ith->getUpperBound();
			ith++;
		}
		if (min!= std::numeric_limits<double>::infinity() && max!= - std::numeric_limits<double>::infinity() && !checkInterval(result,Bounds(min,max)))
		{
			result.push_back(Bounds(min,max));
		}
		ita++;
	}
	return result;
}

std::vector<Bounds> LoosePathConsistency::reverseIntervals(std::vector<Bounds> v)
{
	std::vector<Bounds> result;
	std::vector<Bounds>::iterator it = v.begin();
	while (it!=v.end()) 
		{
			result.push_back(Bounds(-(it->getUpperBound()), -(it->getLowerBound())));
			it++;
		}
	return result;
}

TemporalConstraintNetwork LoosePathConsistency::intersectionNetwork(TemporalConstraintNetwork t)
{
	TemporalConstraintNetwork tcn;
	graph_analysis::BaseGraph::Ptr graph = t.getDistanceGraph();
	graph_analysis::EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
	while (edgeIt->next())
	{
		IntervalConstraint::Ptr edge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
		graph_analysis::BaseGraph::Ptr graph2 = t.getDistanceGraph();
		graph_analysis::VertexIterator::Ptr vit = graph2->getVertexIterator();
		std::vector<Bounds> final,aux;
		int ok = 0;
		int cnt=0;
		while (vit->next())
		{
			cnt = 0;
			point_algebra::TimePoint::Ptr vertex = boost::dynamic_pointer_cast<point_algebra::TimePoint>( vit->current() );
 			if (vertex != edge->getSourceTimePoint() && vertex != edge->getTargetTimePoint())
 			{
 				std::vector<Bounds> a,b;
 				std::vector<graph_analysis::Edge::Ptr> x,y,z,p;
 				IntervalConstraint::Ptr i1,i2;

 				x = graph->graph_analysis::BaseGraph::getEdges(vertex,edge->getSourceTimePoint());
 				if (x.size()==1) 
 				{
 					i1 = boost::dynamic_pointer_cast<IntervalConstraint>(*x.begin());
 					a = reverseIntervals(i1->getIntervals());
 					cnt++;
 				}
 				
 				y = graph->graph_analysis::BaseGraph::getEdges(edge->getSourceTimePoint(),vertex);
 				if (y.size()==1) 
 				{
 					i1 = boost::dynamic_pointer_cast<IntervalConstraint>(*y.begin());
 					a = i1->getIntervals();
 					cnt++;
 				}
 				
 				z = graph->graph_analysis::BaseGraph::getEdges(vertex,edge->getTargetTimePoint());
 				if (z.size()==1) 
 				{
 					i2 = boost::dynamic_pointer_cast<IntervalConstraint>(*z.begin());
 					b = i2->getIntervals();
 					cnt++;
 				}

 				p = graph->graph_analysis::BaseGraph::getEdges(edge->getTargetTimePoint(),vertex);
 				if (p.size()==1)
 				{
 					i2 = boost::dynamic_pointer_cast<IntervalConstraint>(*p.begin());
 					b = reverseIntervals(i2->getIntervals());
 					cnt++;
 				}

 				aux = composition(a,b); 
 			}
 			if (ok==0 && cnt!=0)
 			{
 				final = aux;
 				ok = 1;
 			}
 			else
 			{
 				if (cnt!=0) final = intersection(final,aux);
 			}

		}
		//return cnt;
		std::vector<Bounds>::iterator it = final.begin();
		IntervalConstraint::Ptr i(new IntervalConstraint(edge->getSourceTimePoint(), edge->getTargetTimePoint()));
		while (it!=final.end())
		{
			i->addInterval(*it);
			it++;
		}
		tcn.addIntervalConstraint(i);
	}
	return tcn;
	//return 0;
}

TemporalConstraintNetwork LoosePathConsistency::looseNetwork(TemporalConstraintNetwork a, TemporalConstraintNetwork b)
{
	TemporalConstraintNetwork tcn;
	graph_analysis::BaseGraph::Ptr graph = a.getDistanceGraph();
	graph_analysis::BaseGraph::Ptr graph2 = b.getDistanceGraph();

	graph_analysis::EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
	
	while (edgeIt->next())
	{
		IntervalConstraint::Ptr edge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
		std::vector<Bounds> final,aux1,aux2;
		aux1 = edge->getIntervals();

		std::vector<graph_analysis::Edge::Ptr> x;
		x = graph2->graph_analysis::BaseGraph::getEdges(edge->getSourceTimePoint(),edge->getTargetTimePoint());
		
		IntervalConstraint::Ptr i = boost::dynamic_pointer_cast<IntervalConstraint>(*x.begin());
 		aux2 = i->getIntervals();

 		final = looseIntersection(aux1,aux2); 

 		std::vector<Bounds>::iterator it = final.begin();
		IntervalConstraint::Ptr t(new IntervalConstraint(edge->getSourceTimePoint(), edge->getTargetTimePoint()));
		while (it!=final.end())
		{
			t->addInterval(*it);
			it++;
		}
		tcn.addIntervalConstraint(t);
	}
	
	return tcn;
}

TemporalConstraintNetwork LoosePathConsistency::loosePathConsistency(TemporalConstraintNetwork t)
{
	TemporalConstraintNetwork tcn;
	return tcn;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ