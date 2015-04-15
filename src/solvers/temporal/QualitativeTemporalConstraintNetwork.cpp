#include "QualitativeTemporalConstraintNetwork.hpp"
#include <numeric/Combinatorics.hpp>

using namespace templ::solvers::temporal::point_algebra;
using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace temporal {

QualitativeTemporalConstraintNetwork::QualitativeTemporalConstraintNetwork() 
    : TemporalConstraintNetwork()
{}

void QualitativeTemporalConstraintNetwork::addQualitativeConstraint(TimePoint::Ptr t1, TimePoint::Ptr t2, QualitativeConstraintType constraintType)
{
    // Add a qualitative constraint -- translate "complexer" type to primitive
    // relations
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

bool QualitativeTemporalConstraintNetwork::isConsistent(std::vector<Edge::Ptr> edges)
{
    if(edges.size() <= 1)
    {
        return true;
    }

    numeric::Combination<Edge::Ptr> combination(edges, 2, numeric::EXACT);
    do {

        std::vector<Edge::Ptr> edgeCombination = combination.current();
        QualitativeConstraintType constraintTypeIJ = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgeCombination[0])->getQualitativeConstraintType();
        QualitativeConstraintType constraintTypeJI = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgeCombination[1])->getQualitativeConstraintType();

        if(! QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, constraintTypeJI) )
        {
            return false;
        }
    } while(combination.next());

    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent(Vertex::Ptr vertex0, Vertex::Ptr vertex1, BaseGraph::Ptr graph)
{
    int i = 0;
    int j = 1;
    std::vector<Edge::Ptr> edgesIJ = graph->getEdges(vertex0, vertex1);
    std::vector<Edge::Ptr> edgesJI = graph->getEdges(vertex1, vertex0);
    // Check self consistency
    if(!isConsistent(edgesIJ) || !isConsistent(edgesJI))
    {
        return false;
    }

    std::vector<Edge::Ptr>::const_iterator ijIterator = edgesIJ.begin();
    for(; ijIterator != edgesIJ.end(); ++ijIterator)
    {
        std::vector<Edge::Ptr>::const_iterator jiIterator = edgesJI.begin();
        for(; jiIterator != edgesJI.end(); ++jiIterator)
        {
            QualitativeConstraintType constraintTypeIJ = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(*ijIterator)->getQualitativeConstraintType();

            QualitativeConstraintType constraintTypeJI = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(*jiIterator)->getQualitativeConstraintType();
            QualitativeConstraintType symmetricConstraintTypeJI = QualitativeTimePointConstraint::getSymmetric(constraintTypeJI);

            if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, symmetricConstraintTypeJI))
            {
                LOG_DEBUG_S << "Constraint: " << QualitativeConstraintTypeTxt[constraintTypeIJ] << " is inconsistent with " << QualitativeConstraintTypeTxt[symmetricConstraintTypeJI];
                return false;
            }
        }
    }
    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent()
{
    using namespace graph_analysis;
    BaseGraph::Ptr graph = mDigraph.copy();

    // Path consistency
    // if there is no constraint between two variables, we assume the
    // universal constraint

    // Iterate over all node triples
    std::vector<Vertex::Ptr> vertices = graph->getAllVertices();
    LOG_DEBUG_S << "Vertices in STN: " << vertices.size();
    switch(vertices.size())
    {
        case 0:
        case 1:
            return true;
        case 2:
            return isConsistent(vertices[0], vertices[1], graph);
        default:
            break;
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
            throw std::runtime_error("QualitativeTemporalConstraintNetwork::isConsistent: assuming one directional constraint");
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
                LOG_DEBUG_S << "QualitativeTemporalConstraintNetwork::isConsistent: IJ symmetry assumption does not hold";
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
                LOG_DEBUG_S << "QualitativeTemporalConstraintNetwork::isConsistent: IK symmetry assumption does not hold";
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
                LOG_DEBUG_S << "QualitativeTemporalConstraintNetwork::isConsistent: JK symmetry assumption does not hold";
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

QualitativeConstraintType QualitativeTemporalConstraintNetwork::getSymmetricConstraint(graph_analysis::BaseGraph::Ptr graph, graph_analysis::Vertex::Ptr first, graph_analysis::Vertex::Ptr second)
{
    std::vector<graph_analysis::Edge::Ptr> edges = graph->getEdges(first, second);
    if(edges.empty())
    {
        LOG_DEBUG_S << "QualitativeTemporalConstraintNetwork::getSymmetricConstraint: no edge return Universal";
        return Universal;
    } else if(edges.size() > 1)
    {
        throw std::runtime_error("QualitativeTemporalConstraintNetwork::isConsistent: more than one directional constraint");
    } else {
        QualitativeTimePointConstraint::Ptr ptr = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edges[0]);
        if(!ptr)
        {
            throw std::invalid_argument("QualitativeTemporalConstraintNetwork::getSymmetricConstraint: edge is not a constraint");
        }

        QualitativeConstraintType constraintType = ptr->getQualitativeConstraintType();
        LOG_DEBUG_S << "QualitativeConstraintType: " << QualitativeConstraintTypeTxt[constraintType];
        return QualitativeTimePointConstraint::getSymmetric(constraintType);
    }
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
