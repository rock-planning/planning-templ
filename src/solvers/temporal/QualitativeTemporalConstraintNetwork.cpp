#include "QualitativeTemporalConstraintNetwork.hpp"
#include <base/Logging.hpp>
#include <base/Time.hpp>
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

QualitativeTemporalConstraintNetwork::Ptr QualitativeTemporalConstraintNetwork::complete()
{
    QualitativeTemporalConstraintNetwork::Ptr completeQTCN(new QualitativeTemporalConstraintNetwork());

    if(getGraph()->empty())
    {
        return completeQTCN;
    }

    std::vector<Vertex::Ptr> vertices = getGraph()->getAllVertices();
    size_t numberOfVertices = vertices.size();

    // Check all vertex combinations and verify the bidirectional consistency of the constraint type
    // return a complete network that
    for(size_t i = 0; i < numberOfVertices - 1; ++i)
    {
        for(size_t j = i + 1; j < numberOfVertices; ++j)
        {
            const TimePoint::Ptr& vertex_i = boost::dynamic_pointer_cast<TimePoint>(vertices[i]);
            const TimePoint::Ptr& vertex_j = boost::dynamic_pointer_cast<TimePoint>(vertices[j]);

            QualitativeTimePointConstraint::Type constraintType = getBidirectionalConstraintType(vertex_i, vertex_j);

            LOG_DEBUG_S << "Add qualitative constraint: " << vertex_i->toString() << " -- " << vertex_j->toString() << ": " << QualitativeTimePointConstraint::TypeTxt[constraintType];
            completeQTCN->addQualitativeConstraint(vertex_i, vertex_j, constraintType);
        }
    }
    return completeQTCN;
}

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getQualitativeConstraint(const TimePoint::Ptr& t1, const TimePoint::Ptr& t2)
{
    QualitativeTimePointConstraint::Type type = getBidirectionalConstraintType(t1, t2);
    LOG_DEBUG_S << "Return bidirectionalconstraint type for '"
        << t1->toString() << " --> " << t2->toString() << " : "
        << QualitativeTimePointConstraint::TypeTxt[type];
    return type;
}

QualitativeTimePointConstraint::Ptr QualitativeTemporalConstraintNetwork::addQualitativeConstraint(const TimePoint::Ptr& t1, const TimePoint::Ptr& t2, QualitativeTimePointConstraint::Type constraintType)
{
    if(t1 == t2)
    {
        throw std::runtime_error("templ::solvers::temporal::QualitativeTimePointConstraint::addQualitativeConstraint: cannot add constraint on identical timepoints");
    }

    // compact existing edges
    try {
        std::vector<Edge::Ptr> edgesIJ = getGraph()->getEdges(t1,t2);
        if(!edgesIJ.empty())
        {
            std::vector<Edge::Ptr>::const_iterator eit = edgesIJ.begin();
            for(; eit != edgesIJ.end(); ++eit)
            {
                QualitativeTimePointConstraint::Ptr constraint = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(*eit);
                QualitativeTimePointConstraint::Type existingType;
                if(constraint)
                {
                    existingType = constraint->getType();
                    // merge with existing
                    constraintType = QualitativeTimePointConstraint::getComposition(existingType, constraintType);
                }
                getGraph()->removeEdge(*eit);
            }
        }
    } catch(const std::runtime_error& e)
    {
        // timepoints not yet registered
    }

    if(constraintType == QualitativeTimePointConstraint::Empty)
    {
        throw std::runtime_error("FAILURE");
    }

    QualitativeTimePointConstraint::Ptr constraint = QualitativeTimePointConstraint::create(t1, t2, constraintType);
    LOG_DEBUG_S << "Adding QualitativeTimePointConstraint: " << constraint->toString();
    TemporalConstraintNetwork::addConstraint(constraint);
    mUpdatedConstraints.push_back(VertexPair(t1,t2));

    return constraint;
}


void QualitativeTemporalConstraintNetwork::removeQualitativeConstraint(const point_algebra::QualitativeTimePointConstraint::Ptr& constraint)
{
    removeConstraint(constraint);
}

bool QualitativeTemporalConstraintNetwork::isConsistent()
{
    try {
        return incrementalPathConsistency();
    } catch(const std::runtime_error& e)
    {
        return false;
    }
}

bool QualitativeTemporalConstraintNetwork::incrementalPathConsistency()
{
    size_t count = 0;
    std::vector<Vertex::Ptr> vertices = getGraph()->getAllVertices();
    LOG_DEBUG_S << "Incremental path consistency: updated constraints #" << mUpdatedConstraints.size();
    while(!mUpdatedConstraints.empty())
    {
        VertexPair pair = mUpdatedConstraints.back();
        mUpdatedConstraints.pop_back();

        std::vector<Vertex::Ptr>::const_iterator cit = vertices.begin();
        for(; cit != vertices.end(); ++cit)
        {
            const Vertex::Ptr& k = *cit;
            const Vertex::Ptr& i = pair.first;
            const Vertex::Ptr& j = pair.second;

            if(k == i || k == j || i == j)
            {
                // not a triangle thus continue
                continue;
            }

            LOG_WARN_S << "Checking for: " << k->toString() << "-" << i->toString() << "-" << j->toString();


            VertexPair ik = revise(i, k, composition(i,j,k));
            if(ik != VertexPair())
            {
                LOG_WARN_S << "Updated constraint for " << i->toString() << " -- " << k->toString();
                mUpdatedConstraints.push_back(ik);
            }

            VertexPair jk = revise(j, k, composition(j,i,k));
            if(jk != VertexPair())
            {
                LOG_WARN_S << "Updated constraint for " << j->toString() << " -- " << k->toString();
                mUpdatedConstraints.push_back(jk);
            }
            VertexPair ij = revise(i, j, composition(i,k,j));
        }

        LOG_WARN_S << "Incremental path consistency iteration #" << ++count;
    }

    return true;
}

// c_ij o c_jk
QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::composition(const Vertex::Ptr& i, const Vertex::Ptr& j, const Vertex::Ptr& k)
{
    QualitativeTimePointConstraint::Type constraintTypeIJ = getBidirectionalConstraintType(i,j);
    QualitativeTimePointConstraint::Type constraintTypeJK = getBidirectionalConstraintType(j,k);

    return QualitativeTimePointConstraint::getComposition(constraintTypeIJ, constraintTypeJK);
}


QualitativeTemporalConstraintNetwork::VertexPair QualitativeTemporalConstraintNetwork::revise(const Vertex::Ptr& i, const Vertex::Ptr& j, QualitativeTimePointConstraint::Type pathConstraintType)
{
    QualitativeTimePointConstraint::Type ij = QualitativeTemporalConstraintNetwork::getBidirectionalConstraintType(i,j);
    QualitativeTimePointConstraint::Type intersectionType = QualitativeTimePointConstraint::getIntersection(ij, pathConstraintType);
    
    LOG_DEBUG_S << "Revise: " << QualitativeTimePointConstraint::TypeTxt[pathConstraintType] << ":  and bidirectional: " << QualitativeTimePointConstraint::TypeTxt[ij] << " --> " 
        << QualitativeTimePointConstraint::TypeTxt[intersectionType];

    if(intersectionType == QualitativeTimePointConstraint::Empty)
    {
        throw std::runtime_error("QualitativeTimePointConstraint::revise: inconsistent constraint: '" + i->toString() + "' - '" + j->toString());
    }

    if(intersectionType == ij)
    {
        return VertexPair();
    }

    std::vector<Edge::Ptr> edges = getGraph()->getEdges(i, j);
    if(!edges.empty())
    {
        QualitativeTimePointConstraint::Ptr constraint = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edges[0]);
        constraint->setType(intersectionType);
    } else {
        addQualitativeConstraint(boost::dynamic_pointer_cast<TimePoint>(i), boost::dynamic_pointer_cast<TimePoint>(j), intersectionType);
    }

    return VertexPair(i,j);
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

//void QualitativeTemporalConstraintNetwork::minNetwork()
//{
//    if(!QualitativeTemporalConstraintNetwork::isConsistent())
//    {
//        return false;
//    }
//
//    std::vector<Vertex::Ptr> vertices = getGraph()->getAllVertices();
//    getGraph()->clearEdges();
//
//    if(vertices.size() > 1)
//    {
//        numeric::Combination<Vertex::Ptr> combination(vertices, 2, numeric::EXACT);
//        do {
//            std::vector<Vertex::Ptr> pair = combination.current();
//            Vertex::Ptr i = pair[0];
//            Vertex::Ptr j = pair[1];
//
//            std::pair<Vertex::Ptr, Vertex::Ptr> IJ(i,j);
//            std::pair<Vertex::Ptr, Vertex::Ptr> JI(j,i);
//
//            addQualitativeConstraint(i,j, mCurrentUpdatedConstraints[IJ]);
//            addQualitativeConstraint(j,i, mCurrentUpdatedConstraints[JI]);
//        } while(!combination.next());
//    }
//
//    return true;
//}

bool QualitativeTemporalConstraintNetwork::isConsistent(const std::vector<Vertex::Ptr>& vertexTriangle)
{
    // When no constraint update has been performed for constraints
    // in this triangle, then it can be assumed to be still consistent
    //
    // Otherwise a previous run did determine inconsistency
    const Vertex::Ptr& v0 = vertexTriangle[0];
    const Vertex::Ptr& v1 = vertexTriangle[1];
    const Vertex::Ptr& v2 = vertexTriangle[2];

    int count = 0;
    base::Time start = base::Time::now();

//    numeric::Permutation<Vertex::Ptr> permutation(vertexTriangle);
//    do {
//        base::Time intermediateStart = base::Time::now();
//
//        std::vector<Vertex::Ptr> triangle = permutation.current();
        std::vector<Vertex::Ptr> triangle = vertexTriangle;

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
            ++count;
            if(compositionType == QualitativeTimePointConstraint::Empty)
            {
                return false;
            }
        } catch(const std::runtime_error& e)
        {
            LOG_INFO_S << e.what();
            return false;
        }
        //LOG_WARN_S << "Time for triangle permutation: " << intermediate.toSeconds();
    //} while(permutation.next());

    LOG_WARN_S << "Number of checks to verify consistency: " << count  << " time needed: " << (start - base::Time::now()).toSeconds();

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
    if(edgesIJ.empty())
    {
        LOG_DEBUG_S << "No edge between: " << i->toString() << " and " << j->toString();
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

        LOG_DEBUG_S << "Intersection of constraints from: " << i->toString() << " to " << j->toString() << std::endl
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
    QualitativeTimePointConstraint::Type constraintTypeJI = getDirectionalConstraintType(j,i);

    QualitativeTimePointConstraint::Type constraintTypeJI_sym = QualitativeTimePointConstraint::getSymmetric(constraintTypeJI);

    LOG_DEBUG_S << "IJ: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << " and JI" << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI] << " (JI' " << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI_sym];

    if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, constraintTypeJI_sym))
    {
        LOG_DEBUG_S << "IJ symmetry assumption does not hold" <<
            " ij: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << " vs. ji' " << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI_sym];
        throw std::runtime_error("QualitativeTimePointConstraint::getBidirectionalConstraintType: IJ - JI with inconsistent constraints: '" + i->toString() + "' - '" + j->toString());
    }

    LOG_INFO_S << "Checking bidirectional type for: " << i->toString() << " -- " << j->toString();
    QualitativeTimePointConstraint::Type intersectionType = QualitativeTimePointConstraint::getIntersection(constraintTypeIJ, constraintTypeJI_sym);

    LOG_DEBUG_S << "Intersection of IJ (" << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << ")"
        << " and JI' (symmetric IJ) (" << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI_sym] << ")"
        << " --> " << QualitativeTimePointConstraint::TypeTxt[intersectionType];

    return intersectionType;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
