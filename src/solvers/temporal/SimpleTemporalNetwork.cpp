#include "SimpleTemporalNetwork.hpp"
#include <numeric/Combinatorics.hpp>
#include <base/Logging.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/algorithms/FloydWarshall.hpp>

using namespace templ::solvers::temporal::point_algebra;
using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace temporal {

SimpleTemporalNetwork::SimpleTemporalNetwork()
    : TemporalConstraintNetwork()
{
}

void SimpleTemporalNetwork::addInterval(TimePoint::Ptr source, TimePoint::Ptr target, const Bounds& bounds)
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

bool SimpleTemporalNetwork::isConsistent()
{
    return !hasNegativeCycle();
}

bool SimpleTemporalNetwork::hasNegativeCycle()
{
    try {
        // Throw when the shortest path computation identifies a negative cycle,
        // i.e. an inconsistent network whose constraints can never be fulfilled
        bool throwOnNegativeCycle = true;
        algorithms::DistanceMatrix distanceMatrix = algorithms::FloydWarshall::allShortestPaths(mpDistanceGraph, [](Edge::Ptr e) -> double
                {
                    return dynamic_pointer_cast<WeightedEdge>(e)->getWeight();
                }, throwOnNegativeCycle);

        return false;
    } catch(...)
    {
        return true;
    }

}

graph_analysis::BaseGraph::Ptr SimpleTemporalNetwork::propagate()
{
    using namespace graph_analysis;
    
    if(hasNegativeCycle())
    {
        throw std::runtime_error("templ::solvers::temporal::SimpleTemporalNetwork::propagate: cannot progate on a network with negative cycle");
    }

    // Now we should update the vertices/ Variable domains according to the minimal network
    // Since the minimal distance has been already computed, which have to do
    // this once only
    EdgeIterator::Ptr edgeIt = mpDistanceGraph->getEdgeIterator();
    while(edgeIt->next())
    {
        WeightedEdge::Ptr edge = dynamic_pointer_cast<WeightedEdge>( edgeIt->current() );
        double weight = edge->getWeight();

        TimePoint::Ptr sourceTp = dynamic_pointer_cast<TimePoint>(edge->getSourceVertex());
        TimePoint::Ptr targetTp = dynamic_pointer_cast<TimePoint>(edge->getTargetVertex());

        // Update
        //    |------- 38 ----->
        // v0[10,80]          v1[30,100]
        //    <------ -30 -----|
        // ==>
        // v0[10,70] --- v1[48,100]
        //
        // Reading material: Temporal constraints networks, Rina Dechter et. al 1991 p. 70
        //
        if(weight > 0)
        {
            LOG_DEBUG_S << "[" << sourceTp->getLowerBound() <<"," << sourceTp->getUpperBound() <<"] -- " << weight << " --> " << "[" << targetTp->getLowerBound() << "," << targetTp->getUpperBound() << "]";

            double transitiveLowerBound = static_cast<double>(sourceTp->getLowerBound()) + weight;
            double actualLowerBound = static_cast<double>(targetTp->getLowerBound());
            double max = std::max(transitiveLowerBound, actualLowerBound);
            LOG_DEBUG_S << "Lower bounds: transitive: " << transitiveLowerBound << " vs. actual " << actualLowerBound;
            targetTp->setLowerBound(max);
        } else {
            LOG_DEBUG_S << "[" << sourceTp->getLowerBound() <<"," << sourceTp->getUpperBound() <<"] -- " << weight << " --> " << "[" << targetTp->getLowerBound() << "," << targetTp->getUpperBound() << "]";

            double transitiveUpperBound = static_cast<double>(sourceTp->getUpperBound()) + weight;
            double actualUpperBound = static_cast<double>(targetTp->getUpperBound());
            double min = std::min(transitiveUpperBound, actualUpperBound);
            LOG_DEBUG_S << "Upper bounds: transitive: " << transitiveUpperBound << " vs. actual " << actualUpperBound;
            targetTp->setUpperBound(min);
        }
    }

    return mpDistanceGraph;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
