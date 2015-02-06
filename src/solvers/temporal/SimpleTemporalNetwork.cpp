#include "SimpleTemporalNetwork.hpp"
#include <numeric/Combinatorics.hpp>
#include <base/Logging.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/algorithms/FloydWarshall.hpp>

using namespace templ::solvers::temporal::point_algebra;

namespace templ {
namespace solvers {
namespace temporal {

SimpleTemporalNetwork::SimpleTemporalNetwork()
    : mpDistanceGraph( new graph_analysis::lemon::DirectedGraph() )
{
}
void SimpleTemporalNetwork::addTimePoint(TimePoint::Ptr t)
{
    mpDistanceGraph->addVertex(t);
}

void SimpleTemporalNetwork::addInterval(TimePoint::Ptr source, TimePoint::Ptr target, const Bounds& bounds)
{
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

void SimpleTemporalNetwork::addQualitativeConstraint(TimePoint::Ptr t1, TimePoint::Ptr t2, QualitativeConstraintType constraintType)
{
    QualitativeTimePointConstraint::Ptr constraint;
    switch(constraintType)
    {
        case GreaterOrEqual:
            constraint = QualitativeTimePointConstraint::create(t2, t1, Less);
        case LessOrEqual:
            constraint = QualitativeTimePointConstraint::create(t2, t1, Greater);
        default:
            constraint = QualitativeTimePointConstraint::create(t1, t2, constraintType);
    }

    addConstraint(constraint);
}

bool SimpleTemporalNetwork::isConsistent()
{
    using namespace graph_analysis;
    BaseGraph::Ptr graph = mDigraph.copy();

    // Path consistency
    // if there is no constraint between two variables, we assume the
    // universal constraint

    // Iterate over all node triples
    std::vector<Vertex::Ptr> vertices = graph->getAllVertices();
    LOG_WARN_S << "Vertices count: " << vertices.size();
    if(vertices.size() < 3)
    {
        return true;
    }

    numeric::Combination<Vertex::Ptr> combination(vertices, 3, numeric::EXACT);
    do {
        std::vector<Vertex::Ptr> triangle = combination.current();

        int i = 0;
        int j = 2;
        int k = 1;

        std::vector<Edge::Ptr> edgesIJ = graph->getEdges(triangle[i], triangle[j]);
        std::vector<Edge::Ptr> edgesIK = graph->getEdges(triangle[i], triangle[k]);
        std::vector<Edge::Ptr> edgesKJ = graph->getEdges(triangle[k], triangle[j]);

        if (edgesIK.size() > 1 || edgesKJ.size() > 1 || edgesIK.size() > 1)
        {
            throw std::runtime_error("SimpleTemporalNetwork::isConsistent: assuming one directional constraint");
        }

        // Universal constraint -- if there is no edge defined
        // IJ -- Constraint between nod I and J
        QualitativeConstraintType constraintTypeIJ = Empty;
        QualitativeConstraintType constraintTypeJI = getSymmetricConstraint(graph, triangle[j], triangle[i]);
        if(edgesIJ.empty())
        {
            constraintTypeIJ = constraintTypeJI;
        } else {
            constraintTypeIJ = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgesIJ[0])->getQualitativeConstraintType();
            if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, constraintTypeJI))
            {
                LOG_DEBUG_S << "SimpleTemporalNetwork::isConsistent: IJ symmetry assumption does not hold";
                return false;
            }
        }
        LOG_DEBUG_S << "IJ " << QualitativeConstraintTypeTxt[constraintTypeIJ];
        LOG_DEBUG_S << "JI' " << QualitativeConstraintTypeTxt[constraintTypeJI];

        // IK
        QualitativeConstraintType constraintTypeIK = Empty;
        QualitativeConstraintType constraintTypeKI = getSymmetricConstraint(graph, triangle[k], triangle[i]);
        if(edgesIK.empty())
        {
            constraintTypeIK = constraintTypeKI;
        } else {
            constraintTypeIK = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgesIK[0])->getQualitativeConstraintType();
            if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIK, constraintTypeKI))
            {
                LOG_DEBUG_S << "SimpleTemporalNetwork::isConsistent: IK symmetry assumption does not hold";
                return false;
            }
        }
        LOG_DEBUG_S << "IK " << QualitativeConstraintTypeTxt[constraintTypeIK];
        LOG_DEBUG_S << "KI' " << QualitativeConstraintTypeTxt[constraintTypeKI];

        // KJ
        QualitativeConstraintType constraintTypeKJ = Empty;
        QualitativeConstraintType constraintTypeJK = getSymmetricConstraint(graph, triangle[j], triangle[k]);
        if(edgesKJ.empty())
        {
            constraintTypeKJ = constraintTypeJK;
        } else {
            constraintTypeKJ = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgesKJ[0])->getQualitativeConstraintType();
            if(!QualitativeTimePointConstraint::isConsistent(constraintTypeKJ, constraintTypeKJ) )
            {
                LOG_DEBUG_S << "SimpleTemporalNetwork::isConsistent: JK symmetry assumption does not hold";
                return false;
            }
        }
        LOG_DEBUG_S << "KJ " << QualitativeConstraintTypeTxt[constraintTypeKJ];
        LOG_DEBUG_S << "JK' " << QualitativeConstraintTypeTxt[constraintTypeJK];

        QualitativeConstraintType compositionIJ = QualitativeTimePointConstraint::getComposition(constraintTypeIK, constraintTypeKJ);

        bool isConsistent = QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, compositionIJ);

        if(!isConsistent)
        {
            LOG_DEBUG_S << "IK: " << QualitativeConstraintTypeTxt[constraintTypeIJ] << ", KJ: " << QualitativeConstraintTypeTxt[constraintTypeKJ] << " and IJ " << QualitativeConstraintTypeTxt[constraintTypeIJ] << " [inconsistent]";
            return false;
        } else {
            LOG_DEBUG_S << "IK: " << QualitativeConstraintTypeTxt[constraintTypeIJ] << ", KJ: " << QualitativeConstraintTypeTxt[constraintTypeKJ] << " and IJ " << QualitativeConstraintTypeTxt[constraintTypeIJ] << " [consistent]";
        }
    } while(combination.next());

    return true;
}

QualitativeConstraintType SimpleTemporalNetwork::getSymmetricConstraint(graph_analysis::BaseGraph::Ptr graph, graph_analysis::Vertex::Ptr first, graph_analysis::Vertex::Ptr second)
{
    std::vector<graph_analysis::Edge::Ptr> edges = graph->getEdges(first, second);
    if(edges.empty())
    {
        LOG_DEBUG_S << "SimpleTemporalNetwork::getSymmetricConstraint: no edge return Universal";
        return Universal;
    } else if(edges.size() > 1)
    {
        throw std::runtime_error("SimpleTemporalNetwork::isConsistent: more than one directional constraint");
    } else {
        QualitativeTimePointConstraint::Ptr ptr = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edges[0]);
        if(!ptr)
        {
            throw std::invalid_argument("SimpleTemporalNetwork::getSymmetricConstraint: edge is not a constraint");
        }

        QualitativeConstraintType constraintType = ptr->getQualitativeConstraintType();
        LOG_DEBUG_S << "QualitativeConstraintType: " << QualitativeConstraintTypeTxt[constraintType];
        return QualitativeTimePointConstraint::getSymmetric(constraintType);
    }
}

graph_analysis::BaseGraph::Ptr SimpleTemporalNetwork::propagate()
{
    using namespace graph_analysis;

    // Throw when the shortest path computation identifies a negative cycle,
    // i.e. an inconsistent network whose constraints can never be fulfilled
    bool throwOnNegativeCycle = true;
    algorithms::DistanceMatrix distanceMatrix = algorithms::FloydWarshall::allShortestPaths(mpDistanceGraph, [](Edge::Ptr e) -> double
            {
                return boost::dynamic_pointer_cast<WeightedEdge>(e)->getWeight();
            }, throwOnNegativeCycle);

    // Now we should update the vertices/ Variable domains according to the minimal network
    // Since the minimal distance has been already computed, which have to do
    // this once only
    EdgeIterator::Ptr edgeIt = mpDistanceGraph->getEdgeIterator();
    while(edgeIt->next())
    {
        WeightedEdge::Ptr edge = boost::dynamic_pointer_cast<WeightedEdge>( edgeIt->current() );
        double weight = edge->getWeight();

        TimePoint::Ptr sourceTp = boost::dynamic_pointer_cast<TimePoint>(edge->getSourceVertex());
        TimePoint::Ptr targetTp = boost::dynamic_pointer_cast<TimePoint>(edge->getTargetVertex());

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
