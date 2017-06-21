#include "LoosePathConsistency.hpp"
#include <graph_analysis/BaseGraph.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <templ/SharedPtr.hpp>
#include <limits>
#include <numeric/Combinatorics.hpp>

namespace templ {
namespace solvers {
namespace temporal {

// computes the intersection between two sets of bounds
std::vector<Bounds> LoosePathConsistency::intersection(const std::vector<Bounds>& aBoundsList, const std::vector<Bounds>& bBoundsList)
{
    std::set<Bounds> resultSet;
    for(const Bounds& a : aBoundsList)
    {
        Bounds intersection;
        // check each bound in a with each bound in b for overlap
        for(const Bounds& b : bBoundsList)
        {
            if (a.getLowerBound() <= b.getLowerBound() && a.getUpperBound() >= b.getUpperBound())
            {
                // edge1: [a--------------a]
                // edge0:    [b-------b]
                // => add:   [b-------b]
                intersection = Bounds(b.getLowerBound(), b.getUpperBound());

                //// if an interval is not already in the final set then we add it
                //if (!checkInterval(result,Bounds(b.getLowerBound(),b.getUpperBound())) &&
                //        b.getLowerBound() < b.getUpperBound())
                //{
                //    result.push_back(Bounds(b.getLowerBound(),b.getUpperBound()));
                //}
            } else if (a.getLowerBound() <= b.getLowerBound() && a.getUpperBound() <= b.getUpperBound() && b.getLowerBound() <= a.getUpperBound())
            {
                // edge1: [a--------a]
                // edge0:     [b------------b]
                // => add:    [b----a]
                intersection = Bounds(b.getLowerBound(),a.getUpperBound());
            } else if (a.getLowerBound() >= b.getLowerBound() && a.getUpperBound() >= b.getUpperBound() && b.getUpperBound() >= a.getLowerBound())
            {
                // edge1:        [a-----------a]
                // edge0: [b-------b]
                // => add:   [a----b]
                intersection = Bounds(a.getLowerBound(),b.getUpperBound());
            } else if (a.getLowerBound() >= b.getLowerBound() && a.getUpperBound() <= b.getUpperBound())
            {
                // edge1:     [a----a]
                // edge0:  [b-----------b]
                // => add:    [a----a]
                intersection = Bounds(a.getLowerBound(),a.getUpperBound());
            } else {
                // otherwise: there is no overlap

                //  edge1:  [a----a]
                //  edge0:            [b----b]
                //  or
                //  edge1:            [a----a]
                //  edge0:  [b----b]
                //  => we don't add anything
                continue;
            }

            if(intersection.isValid())
            {
                resultSet.insert(intersection);
            } else {
                LOG_WARN_S << "Intersection is not valid: " << intersection.toString();
            }
        }
    }

    return std::vector<Bounds>(resultSet.begin(), resultSet.end());
}

// check if interval x is included into the set of intervals a
bool LoosePathConsistency::isIncluded(const std::vector<Bounds>& a, const Bounds& x)
{
    for(const Bounds& b : a)
    {
        if (b.getLowerBound() == x.getLowerBound() && b.getUpperBound() == x.getUpperBound())
        {
            return true;
        }
    }
    return false;
}

// computes the composition between two sets of intervals
std::vector<Bounds> LoosePathConsistency::composition(const std::vector<Bounds>& aBoundsList, const std::vector<Bounds>& bBoundsList)
{
    std::set<Bounds> resultSet;
    for(const Bounds& a : aBoundsList)
    {
        // for each interval [x1,y1] in a and each interval [x2,y2] in b, add
        // the interval [x1+x2,y1+y2]
        for(const Bounds& b : bBoundsList)
        {
            Bounds x(a.getLowerBound()+b.getLowerBound(), a.getUpperBound()+b.getUpperBound());
            if(x.isValid())
            {
                resultSet.insert(x);
            }
        }
    }

    std::vector<Bounds> result(resultSet.begin(), resultSet.end());
    return computeDisjointIntervals(result);
}

std::vector<Bounds> LoosePathConsistency::computeDisjointIntervals(const std::vector<Bounds>& _list)
{
    if(_list.empty())
    {
        return _list;
    }

    std::vector<Bounds> list = _list;
    for(size_t i = 0; i < list.size() - 1; ++i)
    {
        Bounds& a = list[i];
        for(size_t d = i + 1; d < list.size();)
        {
            Bounds& b = list[d];

            if(a.overlaps(b))
            {
                a.setLowerBound( std::min(a.getLowerBound(), b.getLowerBound()) );
                a.setUpperBound( std::max(a.getUpperBound(), b.getUpperBound()) );
                list.erase(list.begin() + d);

                --i;
                break;
            } else {
                ++d;
            }
        }
    }
    return list;
}

// computes the loose intersection between two sets of intervals
std::vector<Bounds> LoosePathConsistency::looseIntersection(const std::vector<Bounds>& aBoundsList, const std::vector<Bounds>& bBoundsList)
{
    std::vector<Bounds> result;
    for(const Bounds& a : aBoundsList)
    {
        std::vector<Bounds> temp;
        temp.push_back(a);

        // for each interval in a, computes the intersection between that
        // interval and the set bBoundsList
        std::vector<Bounds> abIntersections = intersection(temp,bBoundsList);

        double min = std::numeric_limits<double>::max();
        double max = std::numeric_limits<double>::lowest();
        // find the lower and upper bounds of the computed intersection and add
        // this interval into result, if it was not already added
        for(const Bounds& ab : abIntersections)
        {
            min = std::min(ab.getLowerBound(), min);
            max = std::max(ab.getUpperBound(), max);
        }

        Bounds intersection(min,max);
        if( min != std::numeric_limits<double>::max()
                && max != std::numeric_limits<double>::min()
                && !isIncluded(result,intersection))
        {
            result.push_back(intersection);
        }
    }
    return result;
}

// computes the intersection between all triangles which have a common edge
TemporalConstraintNetwork LoosePathConsistency::intersectionNetwork(TemporalConstraintNetwork t)
{
    using namespace graph_analysis;
    TemporalConstraintNetwork tcn;
    BaseGraph::Ptr distanceGraph = t.getDistanceGraph();

    // for each edge in the distance graph of the temp.constr.network check any
    // combination between this edge and another vertex from the distance graph
    //
    // effectively this is checking the triangle constraints
    Vertex::PtrList allVertices = distanceGraph->getAllVertices();
    numeric::Combination<Vertex::Ptr> combination(allVertices, 2, numeric::EXACT);
    do
    {
        const Vertex::PtrList& edge = combination.current();
        const point_algebra::TimePoint::Ptr& sourceTimePoint = dynamic_pointer_cast<point_algebra::TimePoint>(edge[0]);
        const point_algebra::TimePoint::Ptr& targetTimePoint = dynamic_pointer_cast<point_algebra::TimePoint>(edge[1]);

        std::vector<Bounds> finalBoundsList, boundsComposition;
        for(const Vertex::Ptr& vertex : allVertices)
        {
            if(vertex == sourceTimePoint || vertex == targetTimePoint)
            {
                continue;
            }

            point_algebra::TimePoint::Ptr timePoint = dynamic_pointer_cast<point_algebra::TimePoint>(vertex);

            // Collecting bounds check if the (timepoint) vertex is not the
            // source or target of our current edge since we need a triangle
            if (timePoint != sourceTimePoint && timePoint != targetTimePoint)
            {
                std::vector<Bounds> a,b;

                /*
                    source --------> target
                      |
                      |
                      |
                      *
                    vertex
                    => we keep the same intervals
                */
                Edge::PtrList y = distanceGraph->getEdges(sourceTimePoint,timePoint);
                if (y.size()==1)
                {
                    IntervalConstraint::Ptr i = dynamic_pointer_cast<IntervalConstraint>(*y.begin());
                    const Bounds::List& intervals = i->getIntervals();
                    a.insert(a.begin(), intervals.begin(), intervals.end());
                }

                //
                //     source -------> target
                //       ^
                //       |
                //       |
                //       |
                //     timePoint
                //     => we need to reverse the intervals corresponding to [timePoint,source]
                //
                // Check for each timepoint the forwared and backward direction
                Edge::PtrList x = distanceGraph->getEdges(timePoint,sourceTimePoint);
                if (x.size()==1)
                {
                    IntervalConstraint::Ptr i = dynamic_pointer_cast<IntervalConstraint>(*x.begin());
                    const Bounds::List& intervals = Bounds::reverse(i->getIntervals());
                    a.insert(a.begin(), intervals.begin(), intervals.end());
                }

                LOG_DEBUG_S << "Bounds: a " << Bounds::toString(a) << " from: "<< sourceTimePoint  << " to: " << timePoint->getLabel();

                /*
                    source --------> target
                            ^
                            |
                            |
                            |
                              vertex
                    => we keep the same intervals
                */
                Edge::PtrList z = distanceGraph->graph_analysis::BaseGraph::getEdges(timePoint,targetTimePoint);
                if (z.size()==1)
                {
                    IntervalConstraint::Ptr i = dynamic_pointer_cast<IntervalConstraint>(*z.begin());
                    const Bounds::List& intervals = i->getIntervals();
                    b.insert(b.begin(), intervals.begin(), intervals.end());
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
                Edge::PtrList p = distanceGraph->getEdges(targetTimePoint,timePoint);
                if (p.size()==1)
                {
                    IntervalConstraint::Ptr i = dynamic_pointer_cast<IntervalConstraint>(*p.begin());
                    const Bounds::List& intervals = Bounds::reverse(i->getIntervals());
                    b.insert(b.begin(), intervals.begin(), intervals.end());
                }
                LOG_DEBUG_S << "Bounds: b " << Bounds::toString(b) << " from: " << timePoint->getLabel() << " to: " << targetTimePoint->getLabel();


                boundsComposition = composition(a,b);

                LOG_DEBUG_S << "Composition: ab " << Bounds::toString(boundsComposition);
            }

            if (finalBoundsList.empty())
            {
                finalBoundsList = boundsComposition;
            } else
            {
                // if we know the values of finalBoundsList and boundsComposition, then we need to
                // compute the intersection between them
                if (!boundsComposition.empty())
                {
                    finalBoundsList = intersection(finalBoundsList, boundsComposition);
                }
            }
        } // end for loop

        IntervalConstraint::Ptr i(new IntervalConstraint(sourceTimePoint, targetTimePoint));
        for(const Bounds& b : finalBoundsList)
        {
            i->addInterval(b);
        }
        tcn.addIntervalConstraint(i);
    } while(combination.next());

    return tcn;
}

// computes the loose intersection between two networks (loose intersection between each pairs of edges)
TemporalConstraintNetwork LoosePathConsistency::looseIntersectionNetwork(const TemporalConstraintNetwork& a, const TemporalConstraintNetwork& b)
{
    using namespace graph_analysis;

    TemporalConstraintNetwork tcn;
    BaseGraph::Ptr graphA = a.getDistanceGraph();
    BaseGraph::Ptr graphB = b.getDistanceGraph();

    // for each edge in the first network, we look for the edge which has the same
    // source and target vertices and compute the loose intersection between their corresponding sets of intervals
    // and add the result into the final network (tcn)
    // effectively this is checking the triangle constraints
    Vertex::PtrList allVertices = graphA->getAllVertices();
    numeric::Combination<Vertex::Ptr> combination(allVertices, 2, numeric::EXACT);
    do
    {
        const Vertex::PtrList& edge = combination.current();
        const point_algebra::TimePoint::Ptr& sourceTimePoint = dynamic_pointer_cast<point_algebra::TimePoint>(edge[0]);
        const point_algebra::TimePoint::Ptr& targetTimePoint = dynamic_pointer_cast<point_algebra::TimePoint>(edge[1]);

        // Retrieve current intervals
        Bounds::List intervalsA;
        {
            std::vector<graph_analysis::Edge::Ptr> edgeListA;
            edgeListA = graphA->graph_analysis::BaseGraph::getEdges(sourceTimePoint,targetTimePoint);

            // direction ok
            if(edgeListA.size() == 1)
            {
                IntervalConstraint::Ptr edge = dynamic_pointer_cast<IntervalConstraint>(edgeListA.front());
                intervalsA = edge->getIntervals();
            }

            edgeListA = graphA->graph_analysis::BaseGraph::getEdges(targetTimePoint, sourceTimePoint);
            if(edgeListA.size() == 1)
            {
                IntervalConstraint::Ptr edge = dynamic_pointer_cast<IntervalConstraint>(edgeListA.front());
                Bounds::List intervals = Bounds::reverse( edge->getIntervals() );
                intervalsA.insert(intervalsA.begin(), intervals.begin(), intervals.end());
            }
        }

        // Retrieve other intervals
        Bounds::List intervalsB;
        {
            std::vector<graph_analysis::Edge::Ptr> edgeList;
            edgeList = graphB->graph_analysis::BaseGraph::getEdges(sourceTimePoint,targetTimePoint);
            if(edgeList.size() == 1)
            {
                IntervalConstraint::Ptr i = dynamic_pointer_cast<IntervalConstraint>(*edgeList.begin());
                intervalsB = i->getIntervals();
            }
            edgeList = graphB->graph_analysis::BaseGraph::getEdges(targetTimePoint,sourceTimePoint);
            if(edgeList.size() == 1)
            {
                IntervalConstraint::Ptr i = dynamic_pointer_cast<IntervalConstraint>(*edgeList.begin());
                Bounds::List intervals = Bounds::reverse(i->getIntervals());
                intervalsB.insert(intervalsB.begin(), intervals.begin(), intervals.end());
            }
        }

        Bounds::List finalBoundsList = intervalsA;
        if(intervalsA.empty())
        {
            // if original graph has no constraints, i.e. the universal
            // constraint, then we use the given constraints through other edges
            finalBoundsList = intervalsB;
        } else {
            // otherwise we are using the looseIntersection
            finalBoundsList = looseIntersection(intervalsA,intervalsB);
        }

        IntervalConstraint::Ptr t;

        if( Bounds::includesNegative(finalBoundsList) )
        {
            t = IntervalConstraint::Ptr(new IntervalConstraint(targetTimePoint, sourceTimePoint));
            finalBoundsList = Bounds::reverse(finalBoundsList);
        } else {
            t = IntervalConstraint::Ptr(new IntervalConstraint(sourceTimePoint, targetTimePoint));
        }

        for(const Bounds& b : finalBoundsList)
        {
            t->addInterval(b);
        }
        tcn.addIntervalConstraint(t);

    } while(combination.next());

    return tcn;
}

// check the size of each edge
// if we find one which is zero, then the network is inconsistent
bool LoosePathConsistency::checkConsistency(TemporalConstraintNetwork n)
{
    graph_analysis::EdgeIterator::Ptr edgeIt = (n.getDistanceGraph())->getEdgeIterator();
    while (edgeIt->next())
    {
        IntervalConstraint::Ptr edge = dynamic_pointer_cast<IntervalConstraint>( edgeIt->current() );
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
TemporalConstraintNetwork LoosePathConsistency::loosePathConsistency(const TemporalConstraintNetwork& _n)
{
    TemporalConstraintNetwork n = _n;
    TemporalConstraintNetwork n1,n2;
    n2 = n;
    graph_analysis::BaseGraph::Ptr graph;
    size_t i = 0;
    do
    {
        n = n2;
        n1 = intersectionNetwork(n);
        n2 = looseIntersectionNetwork(n,n1);
        graph = n2.getDistanceGraph();

        {
            std::stringstream ss;
            ss << "/tmp/templ-test-loose_path_consistency-n1-" << i << ".gexf";
            graph_analysis::io::GraphIO::write(ss.str(), n1.getDistanceGraph());
        }

        {
            std::stringstream ss;
            ss << "/tmp/templ-test-loose_path_consistency-n2-" << i << ".gexf";
            graph_analysis::io::GraphIO::write(ss.str(), n2.getDistanceGraph());
        }


    } while(!n.equals(graph) && checkConsistency(n2));

    return n;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
