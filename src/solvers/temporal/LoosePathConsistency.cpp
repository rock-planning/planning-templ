#include "LoosePathConsistency.hpp"
#include <graph_analysis/BaseGraph.hpp> 
#include <graph_analysis/GraphIO.hpp>

namespace templ {
namespace solvers {
namespace temporal {

// computes the intersection between two sets of bounds
std::vector<Bounds> LoosePathConsistency::intersection(const std::vector<Bounds>& a, const std::vector<Bounds>& b)
{
    std::vector<Bounds>::const_iterator ita = a.begin();
    std::vector<Bounds> result;
    while (ita != a.end())
    {
        std::vector<Bounds>::const_iterator itb = b.begin();
        // check each bound in a with each bound in b
        while (itb != b.end())
        {
            if (ita->getLowerBound() <= itb->getLowerBound() && ita->getUpperBound() >= itb->getUpperBound())
            {
                /* edge1: [x--------------y]
                   edge0:    [a-------b]
                => add:      [a-------b]
                */    

                // if an interval is not already in the final set then we add it
                if (!checkInterval(result,Bounds(itb->getLowerBound(),itb->getUpperBound())) && 
                        itb->getLowerBound() < itb->getUpperBound())
                {   
                    result.push_back(Bounds(itb->getLowerBound(),itb->getUpperBound()));
                }
            } else if (ita->getLowerBound() <= itb->getLowerBound() && ita->getUpperBound() <= itb->getUpperBound())
            {
                /*      edge1: [x------------y]
                    edge0:      [a------------b]
                    => add:     [a-------y] 
                */
                if (itb->getLowerBound() <= ita->getUpperBound())
                {
                    if (!checkInterval(result,Bounds(itb->getLowerBound(),ita->getUpperBound())) &&
                            itb->getLowerBound() < ita->getUpperBound())
                    {
                        result.push_back(Bounds(itb->getLowerBound(),ita->getUpperBound()));
                    }
                }
            } else if (ita->getLowerBound() >= itb->getLowerBound() && ita->getUpperBound() >= itb->getUpperBound())
            {
            /*  edge1:        [x-----------y]
                    edge0:    [a---------b]
                    => add:       [x-----b]
            */
                if (itb->getUpperBound() >= ita->getLowerBound())
                {
                    if (!checkInterval(result,Bounds(ita->getLowerBound(),itb->getUpperBound())) &&
                            ita->getLowerBound() < itb->getUpperBound())
                    {
                        result.push_back(Bounds(ita->getLowerBound(),itb->getUpperBound()));
                    }
                }
            } else if (ita->getLowerBound() >= itb->getLowerBound() && ita->getUpperBound() <= itb->getUpperBound())
            {
            /*      edge1:     [x----y]
                edge0:  [a-----------b]
                => add:    [x----y]
            */
                if (!checkInterval(result,Bounds(ita->getLowerBound(),ita->getUpperBound())) &&                 ita->getLowerBound() < ita->getUpperBound()) 
                {
                    result.push_back(Bounds(ita->getLowerBound(),ita->getUpperBound()));
                }
            }
            // otherwise:
            /*      edge1:  [x----y]
                edge0:            [a----b]
                or
                edge1:            [x----y]
                edge0:  [a----b]
                => we don't add anything
            */
            ++itb;
        }
        ++ita;
    }
    return result;
}

// check if interval x is included into the set of intervals a
bool LoosePathConsistency::checkInterval(const std::vector<Bounds>& a, const Bounds& x)
{
    std::vector<Bounds>::const_iterator it = a.begin();
    while (it!=a.end())
    {
        if (it->getLowerBound() == x.getLowerBound() && it->getUpperBound() == x.getUpperBound())
        {
            return true;
        }
        ++it;
    }
    return false;
}

// computes the composition between two sets of intervals
std::vector<Bounds> LoosePathConsistency::composition(const std::vector<Bounds>& a, const std::vector<Bounds>& b)
{
    std::vector<Bounds> result;
    std::vector<Bounds>::const_iterator ita = a.begin();
    while (ita!=a.end())
    {
        std::vector<Bounds>::const_iterator itb = b.begin();
        // for each interval [x1,y1] in a and each interval [x2,y2] in b, add the interval [x1+x2,y1+y2] 
        while (itb!=b.end())
        {
            Bounds x(ita->getLowerBound()+itb->getLowerBound(), ita->getUpperBound()+itb->getUpperBound());
            // check if the resulting interval was already added
            if (ita->getLowerBound()+itb->getLowerBound() < ita->getUpperBound()+itb->getUpperBound())
            { 
                if (!checkInterval(result,x)) result.push_back(x);
            }
            ++itb;
        }
        ++ita;
    }
    return result;
}

// computes the loose intersection between two sets of intervals
std::vector<Bounds> LoosePathConsistency::looseIntersection(const std::vector<Bounds>& a, const std::vector<Bounds>& b)
{
    std::vector<Bounds> result;
    std::vector<Bounds>::const_iterator ita = a.begin();
    while (ita!=a.end())
    {
        std::vector<Bounds> temp;
        temp.push_back(*ita);
        // for each interval in a, computes the intersection between that interval (taken as a set of bounds) and b 
        std::vector<Bounds> help = intersection(temp,b);
        std::vector<Bounds>::const_iterator ith = help.begin();
        double min,max;
        min = std::numeric_limits<double>::infinity();
        max = - std::numeric_limits<double>::infinity();
        // find the lower and upper bounds of the computed intersection and add this interval into result, if it was            // not already added
        while (ith!=help.end())
        {
            if (min > ith->getLowerBound()) min = ith->getLowerBound();
            if (max < ith->getUpperBound()) max = ith->getUpperBound();
            ++ith;
        }
        if (min!= std::numeric_limits<double>::infinity() && max!= - std::numeric_limits<double>::infinity() &&         !checkInterval(result,Bounds(min,max)))
        {
            result.push_back(Bounds(min,max));
        }
        ++ita;
    }
    return result;
}

// receives an interval and returns the interval reversed 
// [a,b] becomes [-b,-a]
std::vector<Bounds> LoosePathConsistency::reverseIntervals(const std::vector<Bounds>& v)
{
    std::vector<Bounds> result;
    std::vector<Bounds>::const_iterator it = v.begin();
    while (it != v.end()) 
    {
        result.push_back(Bounds(-(it->getUpperBound()), -(it->getLowerBound())));
        ++it;
    }
    return result;
}

// computes the intersection between all triangles which have a common edge
TemporalConstraintNetwork LoosePathConsistency::intersectionNetwork(TemporalConstraintNetwork t)
{
    TemporalConstraintNetwork tcn;
    graph_analysis::BaseGraph::Ptr graph = t.getDistanceGraph();
    graph_analysis::EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
    // for each edge in the distance graph of the temp.constr.network check any combination between this edge and another 
    // vertex from the distance graph
    while (edgeIt->next())
    {
        IntervalConstraint::Ptr edge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
        graph_analysis::BaseGraph::Ptr graph2 = t.getDistanceGraph();
        graph_analysis::VertexIterator::Ptr vit = graph2->getVertexIterator();
        std::vector<Bounds> final,temp;
        int ok = 0;
        int cnt=0;
        while (vit->next())
        {
            cnt = 0;
            point_algebra::TimePoint::Ptr vertex = boost::dynamic_pointer_cast<point_algebra::TimePoint>                    (vit->current() );
            // check if the vertex is not the source or target of our current edge
            if (vertex != edge->getSourceTimePoint() && vertex != edge->getTargetTimePoint())
            {
                std::vector<Bounds> a,b;
                std::vector<graph_analysis::Edge::Ptr> x,y,z,p;
                IntervalConstraint::Ptr i1,i2;
                /*
                    source -------> target
                      ^
                      |
                      |
                      |
                    vertex
                    => we need to reverse the intervals corresponding to [vertex,source]
                */
                x = graph->graph_analysis::BaseGraph::getEdges(vertex,edge->getSourceTimePoint());
                if (x.size()==1) 
                {
                    i1 = boost::dynamic_pointer_cast<IntervalConstraint>(*x.begin());
                    a = reverseIntervals(i1->getIntervals());
                    ++cnt;
                }
                /*
                    source --------> target
                      |
                      |
                      |
                      *
                    vertex
                    => we keep the same intervals
                */
                y = graph->graph_analysis::BaseGraph::getEdges(edge->getSourceTimePoint(),vertex);
                if (y.size()==1) 
                {
                    i1 = boost::dynamic_pointer_cast<IntervalConstraint>(*y.begin());
                    a = i1->getIntervals();
                    ++cnt;
                }
                /*
                    source --------> target
                            ^
                            |
                            |
                            |
                              vertex
                    => we keep the same intervals
                */
                z = graph->graph_analysis::BaseGraph::getEdges(vertex,edge->getTargetTimePoint());
                if (z.size()==1) 
                {
                    i2 = boost::dynamic_pointer_cast<IntervalConstraint>(*z.begin());
                    b = i2->getIntervals();
                    ++cnt;
                }
                /*
                    source --------> target
                            |
                            |
                            |
                            *
                          vertex
                    => we need to reverse the intervals corresponding to [target,vertex]
                */
                p = graph->graph_analysis::BaseGraph::getEdges(edge->getTargetTimePoint(),vertex);
                if (p.size()==1)
                {
                    i2 = boost::dynamic_pointer_cast<IntervalConstraint>(*p.begin());
                    b = reverseIntervals(i2->getIntervals());
                    ++cnt;
                }

                temp = composition(a,b); 
            }
            if (ok==0 && cnt==2)
            {
                final = temp;
                ok = 1;
            } else
            {
                // if we know the values of final and temp, then we need to compute the intersection between                    // them
                if (cnt==2) final = intersection(final,temp);
            }

        }
        std::vector<Bounds>::iterator it = final.begin();
        IntervalConstraint::Ptr i(new IntervalConstraint(edge->getSourceTimePoint(), edge->getTargetTimePoint()));
        while (it!=final.end())
        {
            i->addInterval(*it);
            ++it;
        }
        tcn.addIntervalConstraint(i);
    }
    return tcn;
}

// computes the loose intersection between two networks (loose intersection between each pairs of edges)
TemporalConstraintNetwork LoosePathConsistency::looseNetwork(TemporalConstraintNetwork a, TemporalConstraintNetwork b)
{
    TemporalConstraintNetwork tcn;
    graph_analysis::BaseGraph::Ptr graph = a.getDistanceGraph();
    graph_analysis::BaseGraph::Ptr graph2 = b.getDistanceGraph();

    graph_analysis::EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
    
    // for each edge in the first network, we look for the edge which has the same 
    // soruce and target vertices and compute the loose intersection between their corresponding sets of intervals
    // and add the result into the final network (tcn)
    while (edgeIt->next())
    {
        IntervalConstraint::Ptr edge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
        std::vector<Bounds> final,temp1,temp2;
        temp1 = edge->getIntervals();

        std::vector<graph_analysis::Edge::Ptr> x;
        x = graph2->graph_analysis::BaseGraph::getEdges(edge->getSourceTimePoint(),edge->getTargetTimePoint());
        
        IntervalConstraint::Ptr i = boost::dynamic_pointer_cast<IntervalConstraint>(*x.begin());
        temp2 = i->getIntervals();

        final = looseIntersection(temp1,temp2); 

        std::vector<Bounds>::iterator it = final.begin();
        IntervalConstraint::Ptr t(new IntervalConstraint(edge->getSourceTimePoint(), edge->getTargetTimePoint()));
        while (it!=final.end())
        {
            t->addInterval(*it);
            ++it;
        }
        tcn.addIntervalConstraint(t);
    }
    
    return tcn;
}

// check the size of each edge
// if we find one which is zero, then the network is inconsistent
bool LoosePathConsistency::checkConsistency(TemporalConstraintNetwork n)
{
    graph_analysis::EdgeIterator::Ptr edgeIt = (n.getDistanceGraph())->getEdgeIterator();
    while (edgeIt->next())
    {
        IntervalConstraint::Ptr edge = boost::dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
        std::vector<Bounds> temp = edge->getIntervals();
        if (temp.size()==0)
        {
            return false;
        }
    }
    return true;
}

// main function
// computes the loose path consistency algorithm
TemporalConstraintNetwork LoosePathConsistency::loosePathConsistency(TemporalConstraintNetwork n)
{
    TemporalConstraintNetwork n1,n2;
    n2 = n;
    graph_analysis::BaseGraph::Ptr graph;
    do
    {
        n = n2;
        n1 = intersectionNetwork(n);
        n2 = looseNetwork(n,n1);
        graph = n2.getDistanceGraph();  
    } while (!n.equals(graph) && checkConsistency(n2));
    return n;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
