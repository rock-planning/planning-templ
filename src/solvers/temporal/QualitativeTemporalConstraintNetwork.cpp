#include "QualitativeTemporalConstraintNetwork.hpp"
#include <numeric/Combinatorics.hpp>

#include <lemon/bfs.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>

#include <base/Logging.hpp>

using namespace templ::solvers::temporal::point_algebra;
using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace temporal {

QualitativeTemporalConstraintNetwork::QualitativeTemporalConstraintNetwork()
    : TemporalConstraintNetwork()
{}

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getConstraint(const TimePoint::Ptr& t1, const TimePoint::Ptr& t2)
{
    std::pair<Vertex::Ptr, Vertex::Ptr> key(t1,t2);
    std::map< std::pair<Vertex::Ptr, Vertex::Ptr>, QualitativeTimePointConstraint::Type >::const_iterator cit = mCompositionConstraints.find(key);
    if( cit != mCompositionConstraints.end())
    {
        LOG_DEBUG_S << "Return cached value: " << QualitativeTimePointConstraint::TypeTxt[cit->second];
        return cit->second;
    } else {
        QualitativeTimePointConstraint::Type type = getBidirectionalConstraintType(t1, t2);
        LOG_DEBUG_S << "Return bidirectionalconstraint type for '"
            << t1->toString() << " --> " << t2->toString() << " : "
            << QualitativeTimePointConstraint::TypeTxt[type];
        return type;
    }
}

Constraint::Ptr QualitativeTemporalConstraintNetwork::addQualitativeConstraint(const TimePoint::Ptr& t1, const TimePoint::Ptr& t2, QualitativeTimePointConstraint::Type constraintType)
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
    return constraint;
}

bool QualitativeTemporalConstraintNetwork::isConsistent()
{
    using namespace graph_analysis;

    // Path consistency
    // if there is no constraint between two variables, we assume the
    // universal constraint

    // Iterate over all node triples
    std::vector<Vertex::Ptr> vertices = getGraph()->getAllVertices();
    LOG_DEBUG_S << "Vertices in STN: " << vertices.size();
    switch(vertices.size())
    {
        case 0:
        case 1:
            return true;
        case 2:
            return isConsistent(vertices[0], vertices[1]);
        default:
            break;
    }

    numeric::Combination<Vertex::Ptr> combination(vertices, 3, numeric::EXACT);
    do {
        std::vector<Vertex::Ptr> triangle = combination.current();
        if(!isConsistent(triangle))
        {
            return false;
        }

    } while(combination.next());

    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent(const std::vector<Edge::Ptr>& edges)
{
    if(edges.size() <= 1)
    {
        return true;
    }

    numeric::Combination<Edge::Ptr> combination(edges, 2, numeric::EXACT);
    do {
        std::vector<Edge::Ptr> edgeCombination = combination.current();

        Edge::Ptr e0 = edgeCombination[0];
        Edge::Ptr e1 = edgeCombination[1];

        std::vector<Vertex::Ptr> vertices = graph_analysis::Edge::getInvolvedVertices(e0, e1);
        size_t vertexCount = vertices.size();
        if(vertexCount == 4)
        {
            // edges do not meet, so no possibility to check for consistency
            continue;
        } else if(vertexCount == 3)
        {
            if(!isConsistent(vertices))
            {
                return false;
            }
        } else if(vertexCount == 2)
        {
            if(!isConsistent(vertices[0], vertices[1]))
            {
                return false;
            }
        } else {
            throw std::runtime_error("QualitativeTemporalConstraintNetwork::isConsistent vertexCount for edge combination is 1 -- internal error, must be mininum 2");
        }
    } while(combination.next());

    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent(const std::vector<Vertex::Ptr>& triangle)
{
    int i = 0;
    int j = 2;
    int k = 1;

    try {
        QualitativeTimePointConstraint::Type ij = getBidirectionalConstraintType(triangle[i], triangle[j]);
        QualitativeTimePointConstraint::Type ik = getBidirectionalConstraintType(triangle[i], triangle[k]);
        QualitativeTimePointConstraint::Type kj = getBidirectionalConstraintType(triangle[k], triangle[j]);

        QualitativeTimePointConstraint::Type ij_composition = QualitativeTimePointConstraint::getComposition(ik,kj);

        QualitativeTimePointConstraint::Type compositionType = QualitativeTimePointConstraint::getIntersection(ij, ij_composition);
        if(compositionType != QualitativeTimePointConstraint::Empty)
        {
            std::pair<Vertex::Ptr, Vertex::Ptr> key(triangle[i], triangle[j]);
            mCompositionConstraints[key] = QualitativeTimePointConstraint::getIntersection(ij, ij_composition);
            LOG_DEBUG_S << "Add cache: " << triangle[i]->toString() << " -> " << triangle[j]->toString();
            return true;
        }
    } catch(const std::runtime_error& e)
    {
        LOG_DEBUG_S << e.what();
    }
    return false;
}

bool QualitativeTemporalConstraintNetwork::isConsistent(const graph_analysis::Vertex::Ptr& v0, const graph_analysis::Vertex::Ptr& v1)
{
    try {
        QualitativeTemporalConstraintNetwork::getBidirectionalConstraintType(v0, v1);
        return true;
    } catch(const std::runtime_error& e)
    {
        LOG_DEBUG_S << e.what();
        return false;
    }
}

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getDirectionalConstraintType(const graph_analysis::Vertex::Ptr& i, const graph_analysis::Vertex::Ptr& j) const
{
    if(i == j)
    {
        LOG_WARN_S << "Checking directional constraint for same vertex";
        return QualitativeTimePointConstraint::Universal;
    }

    std::vector<Edge::Ptr> edgesIJ = getGraph()->getEdges(i, j);
    QualitativeTimePointConstraint::Type constraintTypeIJ = QualitativeTimePointConstraint::Universal;

    std::vector<Edge::Ptr>::const_iterator eit = edgesIJ.begin();
    for(; eit != edgesIJ.end(); ++eit)
    {
        QualitativeTimePointConstraint::Type constraintType = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(*eit)->getType();
        QualitativeTimePointConstraint::Type constraintIntersectionType = QualitativeTimePointConstraint::getIntersection(constraintTypeIJ, constraintType);

        LOG_DEBUG_S << "Intersection of " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ]
            << " and " << QualitativeTimePointConstraint::TypeTxt[constraintType]
            << " --> " << QualitativeTimePointConstraint::TypeTxt[constraintIntersectionType];

        constraintTypeIJ = constraintIntersectionType;
    }

    return constraintTypeIJ;
}

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getBidirectionalConstraintType(const graph_analysis::Vertex::Ptr& i, const graph_analysis::Vertex::Ptr& j) const
{
    if(i == j)
    {
        throw std::runtime_error("templ::solvers::temporal::getBidirectionalConstraintType: identical vertex in comparison");
    }

    QualitativeTimePointConstraint::Type constraintTypeIJ = getDirectionalConstraintType(i,j);
    QualitativeTimePointConstraint::Type constraintTypeJI = QualitativeTimePointConstraint::getSymmetric( getDirectionalConstraintType(j,i) );

    if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, constraintTypeJI))
    {
        LOG_DEBUG_S << "IJ symmetry assumption does not hold" <<
            " ij: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << " vs. ji " << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI];
        throw std::runtime_error("QualitativeTimePointConstraint::getBidirectionalConstraintType: IJ - JI with inconsistent constraints: '" + i->toString() + "' - '" + j->toString());
    }

    QualitativeTimePointConstraint::Type compositionType = QualitativeTimePointConstraint::getIntersection(constraintTypeIJ, constraintTypeJI);
    LOG_DEBUG_S << "Intersection of IJ (" << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << ")"
        << " and JI' (symmetric IJ) (" << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI] << ")"
        << " --> " << QualitativeTimePointConstraint::TypeTxt[compositionType];

    return compositionType;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
