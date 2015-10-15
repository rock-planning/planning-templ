#include "QualitativeTemporalConstraintNetwork.hpp"
#include <base/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/VertexTypeManager.hpp>
#include <graph_analysis/EdgeTypeManager.hpp>

using namespace templ::solvers::temporal::point_algebra;
using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace temporal {

QualitativeTemporalConstraintNetwork::QualitativeTemporalConstraintNetwork()
    : TemporalConstraintNetwork()
{
    using namespace graph_analysis;
    try {
        VertexTypeManager* vertexTypeManager = VertexTypeManager::getInstance();
        vertexTypeManager->registerType("QualitativeTimePoint", Vertex::Ptr(new QualitativeTimePoint("default")), true);

        EdgeTypeManager* edgeTypeManager = EdgeTypeManager::getInstance();
        edgeTypeManager->registerType("QualitativeTimePointConstraint", Edge::Ptr(new QualitativeTimePointConstraint(Variable::Ptr(), Variable::Ptr(), QualitativeTimePointConstraint::Empty)));
    } catch(...)
    {
        // type already registered
    }
}

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getQualitativeConstraint(const TimePoint::Ptr& t1, const TimePoint::Ptr& t2)
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

QualitativeTimePointConstraint::Ptr QualitativeTemporalConstraintNetwork::addQualitativeConstraint(const TimePoint::Ptr& t1, const TimePoint::Ptr& t2, QualitativeTimePointConstraint::Type constraintType)
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


void QualitativeTemporalConstraintNetwork::removeQualitativeConstraint(const point_algebra::QualitativeTimePointConstraint::Ptr& constraint)
{
    mCompositionConstraints.clear();
    removeConstraint(constraint);
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

    mConsistencyChecked = false;
    do
    {
        mUpdatedConstraints = mCurrentUpdatedConstraints;
        mCurrentUpdatedConstraints.clear();

        numeric::Combination<Vertex::Ptr> combination(vertices, 3, numeric::EXACT);
        do {
            std::vector<Vertex::Ptr> triangle = combination.current();
            if(!isConsistent(triangle))
            {
                return false;
            }

        } while(combination.next());

        // Mark that initial consistency has been checked, i.e., all
        // constraints should have been updated
        mConsistencyChecked = true;
    } while(!mCurrentUpdatedConstraints.empty());

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

bool QualitativeTemporalConstraintNetwork::isConsistent(const std::vector<Vertex::Ptr>& vertexTriangle)
{
    // When no constraint update has been performed for constraints
    // in this triangle, then it can be assumed to be still consistent
    //
    // Otherwise a previous run did determine inconsistency
    if(mConsistencyChecked && !(constraintUpdated( VertexPair(vertexTriangle[0], vertexTriangle[1]) ) &&
                constraintUpdated(VertexPair(vertexTriangle[0], vertexTriangle[2])) &&
                constraintUpdated(VertexPair(vertexTriangle[1], vertexTriangle[2]))))
    {
        return true;
    }

    numeric::Permutation<Vertex::Ptr> permutation(vertexTriangle);

    do {
        std::vector<Vertex::Ptr> triangle = permutation.current();

        int i = 0;
        int j = 2;
        int k = 1;

        try {
            LOG_DEBUG_S << "Checking triangle: " << triangle[i]->toString() << " -- " << triangle[j]->toString() <<
                " -- " << triangle[k]->toString();
            QualitativeTimePointConstraint::Type ij = getBidirectionalConstraintType(triangle[i], triangle[j]);
            QualitativeTimePointConstraint::Type ik = getBidirectionalConstraintType(triangle[i], triangle[k]);
            QualitativeTimePointConstraint::Type kj = getBidirectionalConstraintType(triangle[k], triangle[j]);

            QualitativeTimePointConstraint::Type ij_composition = QualitativeTimePointConstraint::getComposition(ik,kj);

            QualitativeTimePointConstraint::Type compositionType = QualitativeTimePointConstraint::getIntersection(ij, ij_composition);
            if(compositionType == QualitativeTimePointConstraint::Empty)
            {
                return false;
            }

            if(updateConstraintCache(triangle[i], triangle[j], compositionType) == QualitativeTimePointConstraint::Empty)
            {
                return false;
            }
        } catch(const std::runtime_error& e)
        {
            LOG_DEBUG_S << e.what();
        }
    } while(permutation.next());

    return true;
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
        LOG_INFO_S << "Checking directional constraint for same vertex";
        return QualitativeTimePointConstraint::Universal;
    }

    std::vector<Edge::Ptr> edgesIJ = getGraph()->getEdges(i, j);
    QualitativeTimePointConstraint::Type constraintTypeIJ = QualitativeTimePointConstraint::Universal;

    // Check if there is a cached value -- then use this one
    std::pair<Vertex::Ptr, Vertex::Ptr> key(i,j);
    ConstraintCache::const_iterator cit = mCompositionConstraints.find(key);
    if( cit != mCompositionConstraints.end())
    {
        LOG_DEBUG_S << "Return cached value for: " << i->toString() << " -- " << j->toString() << " --> " << QualitativeTimePointConstraint::TypeTxt[cit->second];
        constraintTypeIJ = cit->second;
    }

    std::vector<Edge::Ptr>::const_iterator eit = edgesIJ.begin();
    for(; eit != edgesIJ.end(); ++eit)
    {
        QualitativeTimePointConstraint::Ptr constraint = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(*eit);
        if(!constraint)
        {
            throw std::invalid_argument("templ::solvers::temporal::QualitativeTemporalConstraintNetwork::getDirectionalConstraintType: "
                    " edge is not a QualitativeTimePointConstraint, but " + (*eit)->getClassName());
        }
        QualitativeTimePointConstraint::Type constraintType = constraint->getType();
        QualitativeTimePointConstraint::Type constraintIntersectionType = QualitativeTimePointConstraint::getIntersection(constraintTypeIJ, constraintType);

        LOG_DEBUG_S << "Intersection between: " << i->toString() << " and " << j->toString() << std::endl
            << "    " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ]
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
        LOG_INFO_S << "Checking bidirectional constraint for same vertex";
        return QualitativeTimePointConstraint::Universal;
    }

    QualitativeTimePointConstraint::Type constraintTypeIJ = getDirectionalConstraintType(i,j);
    QualitativeTimePointConstraint::Type constraintTypeJI = QualitativeTimePointConstraint::getSymmetric( getDirectionalConstraintType(j,i) );

    if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, constraintTypeJI))
    {
        LOG_DEBUG_S << "IJ symmetry assumption does not hold" <<
            " ij: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << " vs. ji " << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI];
        throw std::runtime_error("QualitativeTimePointConstraint::getBidirectionalConstraintType: IJ - JI with inconsistent constraints: '" + i->toString() + "' - '" + j->toString());
    }

    LOG_INFO_S << "Checking bidirectional type for: " << i->toString() << " -- " << j->toString();
    QualitativeTimePointConstraint::Type compositionType = QualitativeTimePointConstraint::getIntersection(constraintTypeIJ, constraintTypeJI);
    LOG_DEBUG_S << "Intersection of IJ (" << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << ")"
        << " and JI' (symmetric IJ) (" << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI] << ")"
        << " --> " << QualitativeTimePointConstraint::TypeTxt[compositionType];

    return compositionType;
}


QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::updateConstraintCache(const graph_analysis::Vertex::Ptr& v1, const graph_analysis::Vertex::Ptr& v2, QualitativeTimePointConstraint::Type constraintType)
{
    QualitativeTimePointConstraint::Type compositionType = constraintType;

    std::pair<Vertex::Ptr, Vertex::Ptr> key(v1, v2);
    ConstraintCache::const_iterator cit = mCompositionConstraints.find(key);
    if(cit != mCompositionConstraints.end())
    {
        QualitativeTimePointConstraint::Type existingConstraint = cit->second;
        compositionType = QualitativeTimePointConstraint::getIntersection(compositionType, cit->second);
        if(existingConstraint != compositionType)
        {
            LOG_DEBUG_S << "Updating cached constraint: " << v1->toString() << " and " << v2->toString()
                << " --> " << QualitativeTimePointConstraint::TypeTxt[compositionType];
            mUpdatedConstraints.push_back(key);
            mCompositionConstraints[key] = compositionType;
        }
    } else {
        mUpdatedConstraints.push_back(key);
        mCompositionConstraints[key] = compositionType;
    }
    return compositionType;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
