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

std::vector<QualitativeTimePointConstraint::Ptr> QualitativeTemporalConstraintNetwork::getConstraints(TimePoint::Ptr t1, TimePoint::Ptr t2)
{
    return getGraph()->getEdges<QualitativeTimePointConstraint>(t1, t2);
}

void QualitativeTemporalConstraintNetwork::addConstraint(TimePoint::Ptr t1, TimePoint::Ptr t2, QualitativeTimePointConstraint::Type constraintType)
{
    // Add a qualitative constraint -- translate "complexer" type to primitive
    // relations
    QualitativeTimePointConstraint::Ptr constraint;
    switch(constraintType)
    {
        case QualitativeTimePointConstraint::GreaterOrEqual:
            constraint = QualitativeTimePointConstraint::create(t2, t1, QualitativeTimePointConstraint::Less);
        case QualitativeTimePointConstraint::LessOrEqual:
            constraint = QualitativeTimePointConstraint::create(t2, t1, QualitativeTimePointConstraint::Greater);
        default:
            constraint = QualitativeTimePointConstraint::create(t1, t2, constraintType);
    }

    TemporalConstraintNetwork::addConstraint(constraint);
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
        QualitativeTimePointConstraint::Type constraintTypeIJ = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgeCombination[0])->getType();
        QualitativeTimePointConstraint::Type constraintTypeJI = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgeCombination[1])->getType();

        if(! QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, constraintTypeJI) )
        {
            return false;
        }
    } while(combination.next());

    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent(Vertex::Ptr vertex0, Vertex::Ptr vertex1, BaseGraph::Ptr graph)
{
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
            QualitativeTimePointConstraint::Type constraintTypeIJ = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(*ijIterator)->getType();

            QualitativeTimePointConstraint::Type constraintTypeJI = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(*jiIterator)->getType();
            QualitativeTimePointConstraint::Type symmetricConstraintTypeJI = QualitativeTimePointConstraint::getSymmetric(constraintTypeJI);

            if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, symmetricConstraintTypeJI))
            {
                LOG_DEBUG_S << "Constraint: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << " is inconsistent with " << QualitativeTimePointConstraint::TypeTxt[symmetricConstraintTypeJI];
                return false;
            }
        }
    }
    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent()
{
    using namespace graph_analysis;
    BaseGraph::Ptr graph = getGraph()->copy();

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
        QualitativeTimePointConstraint::Type constraintTypeIJ = QualitativeTimePointConstraint::Empty;
        QualitativeTimePointConstraint::Type constraintTypeJI = getSymmetricConstraint(graph, triangle[j], triangle[i]);
        if(edgesIJ.empty())
        {
            constraintTypeIJ = constraintTypeJI;
        } else {
            constraintTypeIJ = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgesIJ[0])->getType();
            if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, constraintTypeJI))
            {
                LOG_DEBUG_S << "QualitativeTemporalConstraintNetwork::isConsistent: IJ symmetry assumption does not hold";
                return false;
            }
        }
        LOG_DEBUG_S << "IJ "                 << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ];
        LOG_DEBUG_S << "JI' (symmetric IJ) " << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI];

        // IK
        QualitativeTimePointConstraint::Type constraintTypeIK = QualitativeTimePointConstraint::Empty;
        QualitativeTimePointConstraint::Type constraintTypeKI = getSymmetricConstraint(graph, triangle[k], triangle[i]);
        if(edgesIK.empty())
        {
            constraintTypeIK = constraintTypeKI;
        } else {
            constraintTypeIK = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgesIK[0])->getType();
            if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIK, constraintTypeKI))
            {
                LOG_DEBUG_S << "QualitativeTemporalConstraintNetwork::isConsistent: IK symmetry assumption does not hold";
                return false;
            }
        }
        LOG_DEBUG_S << "IK "                << QualitativeTimePointConstraint::TypeTxt[constraintTypeIK];
        LOG_DEBUG_S << "KI' (symmetric IK)" << QualitativeTimePointConstraint::TypeTxt[constraintTypeKI];

        // KJ
        QualitativeTimePointConstraint::Type constraintTypeKJ = QualitativeTimePointConstraint::Empty;
        QualitativeTimePointConstraint::Type constraintTypeJK = getSymmetricConstraint(graph, triangle[j], triangle[k]);
        if(edgesKJ.empty())
        {
            constraintTypeKJ = constraintTypeJK;
        } else {
            constraintTypeKJ = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edgesKJ[0])->getType();
            if(!QualitativeTimePointConstraint::isConsistent(constraintTypeKJ, constraintTypeKJ) )
            {
                LOG_DEBUG_S << "QualitativeTemporalConstraintNetwork::isConsistent: JK symmetry assumption does not hold";
                return false;
            }
        }
        LOG_DEBUG_S << "KJ "                << QualitativeTimePointConstraint::TypeTxt[constraintTypeKJ];
        LOG_DEBUG_S << "JK' (symmetric JK)" << QualitativeTimePointConstraint::TypeTxt[constraintTypeJK];

        QualitativeTimePointConstraint::Type compositionIJ = QualitativeTimePointConstraint::getComposition(constraintTypeIK, constraintTypeKJ);

        bool isConsistent = QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, compositionIJ);

        if(!isConsistent)
        {
            LOG_DEBUG_S << "IK: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << ", KJ: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeKJ] << " and IJ " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << " [inconsistent]";
            return false;
        } else {
            LOG_DEBUG_S << "IK: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << ", KJ: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeKJ] << " and IJ " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << " [consistent]";
        }
    } while(combination.next());

    return true;
}

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getSymmetricConstraint(graph_analysis::BaseGraph::Ptr graph, graph_analysis::Vertex::Ptr first, graph_analysis::Vertex::Ptr second)
{
    std::vector<graph_analysis::Edge::Ptr> edges = graph->getEdges(first, second);
    if(edges.empty())
    {
        LOG_DEBUG_S << "QualitativeTemporalConstraintNetwork::getSymmetricConstraint: no edge return Universal";
        return QualitativeTimePointConstraint::Universal;
    } else if(edges.size() > 1)
    {
        throw std::runtime_error("QualitativeTemporalConstraintNetwork::isConsistent: more than one directional constraint");
    } else {
        QualitativeTimePointConstraint::Ptr ptr = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edges[0]);
        if(!ptr)
        {
            throw std::invalid_argument("QualitativeTemporalConstraintNetwork::getSymmetricConstraint: edge is not a constraint");
        }

        QualitativeTimePointConstraint::Type constraintType = ptr->getType();
        LOG_DEBUG_S << "QualitativeTimePointConstraint::Type: " << QualitativeTimePointConstraint::TypeTxt[constraintType];
        return QualitativeTimePointConstraint::getSymmetric(constraintType);
    }
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
