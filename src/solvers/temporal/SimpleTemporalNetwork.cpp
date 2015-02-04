#include "SimpleTemporalNetwork.hpp"
#include <numeric/Combinatorics.hpp>
#include <base/Logging.hpp>

using namespace terep::solvers::temporal::point_algebra;

namespace terep {
namespace solvers {
namespace temporal {

TimePoint::Ptr SimpleTemporalNetwork::createTimepoint(uint64_t lowerBound, uint64_t upperBound)
{
    return TimePoint::Ptr( new TimePoint(lowerBound, upperBound) );
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

} // end namespace temporal
} // end namespace solvers
} // end namespace terep
