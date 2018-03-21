#include "QualitativeTemporalConstraintNetwork.hpp"
#include <base-logging/Logging.hpp>
#include <base/Time.hpp>
#include <numeric/Combinatorics.hpp>
#include <graph_analysis/VertexTypeManager.hpp>
#include <graph_analysis/EdgeTypeManager.hpp>

#include "../../SharedPtr.hpp"
#include "../csp/TemporalConstraintNetwork.hpp"

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
    QualitativeTimePointConstraint::Type type = getBidirectionalConstraintType(t1, t2);
    LOG_DEBUG_S << "Return bidirectionalconstraint type for '"
        << t1->toString() << " --> " << t2->toString() << " : "
        << QualitativeTimePointConstraint::TypeTxt[type];
    return type;
}

QualitativeTimePointConstraint::Ptr QualitativeTemporalConstraintNetwork::addQualitativeConstraint(const TimePoint::Ptr& t1, const TimePoint::Ptr& t2, QualitativeTimePointConstraint::Type _constraintType)
{
    QualitativeTimePointConstraint::Type constraintType = _constraintType;
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
                QualitativeTimePointConstraint::Ptr constraint = dynamic_pointer_cast<QualitativeTimePointConstraint>(*eit);
                QualitativeTimePointConstraint::Type existingType;
                if(constraint)
                {
                    existingType = constraint->getType();
                    // merge with existing
                    constraintType = QualitativeTimePointConstraint::getIntersection(existingType, constraintType);
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
        LOG_WARN_S << "The following constraint will be added, invalidating the network: from: " << t1->toString()
            << " to " << t2->toString() << ": " << QualitativeTimePointConstraint::TypeTxt[_constraintType];
    }

    QualitativeTimePointConstraint::Ptr constraint = QualitativeTimePointConstraint::create(t1, t2, constraintType);
    LOG_DEBUG_S << "Adding QualitativeTimePointConstraint: " << constraint->toString(4);
    TemporalConstraintNetwork::addConstraint(constraint);
    mUpdatedConstraints.insert(VertexPair(t1,t2));

    return constraint;
}


void QualitativeTemporalConstraintNetwork::removeQualitativeConstraint(const point_algebra::QualitativeTimePointConstraint::Ptr& constraint)
{
    removeConstraint(constraint);
}

bool QualitativeTemporalConstraintNetwork::isConsistent()
{
    return csp::TemporalConstraintNetwork::isConsistent(*this);

    //try {
    //    return incrementalPathConsistency();
    //    //return pathConsistency_BeekManak();
    //} catch(const std::runtime_error& e)
    //{
    //    return false;
    //}
}

bool QualitativeTemporalConstraintNetwork::pathConsistency_BeekManak()
{
    std::set<VertexPair> edges;
    std::vector<Vertex::Ptr> vertices = getGraph()->getAllVertices();

    numeric::Combination<Vertex::Ptr> combination(vertices, 2, numeric::EXACT);
    do {
        std::vector<Vertex::Ptr> pair = combination.current();
        edges.insert(VertexPair(pair[0],pair[1]));
    } while(combination.next());

    while(!edges.empty())
    {
        VertexPair pair = *edges.begin();
        Vertex::Ptr i = pair.first;
        Vertex::Ptr j = pair.second;

        edges.erase(edges.begin());

        std::vector<Vertex::Ptr>::const_iterator cit = vertices.begin();
        for(; cit != vertices.end(); ++cit)
        {
            Vertex::Ptr k = *cit;
            if(k == i || k == j)
            {
                continue;
            }

            {
                ConstraintValidationResult constraint = getConstraintType(i,j,k);
                if(constraint.st_final != constraint.st)
                {
                    constraint.constraint_st->setType(constraint.st_final);
                    // enforce symmetry
                    QualitativeTimePointConstraint::Ptr ki = getBidirectionalConstraint(k,i);

                    if(ki->getType() == QualitativeTimePointConstraint::Empty)
                    {
                        return false;
                    } else {
                        edges.insert(VertexPair(i,k));
                    }
                } else if(constraint.st_final == QualitativeTimePointConstraint::Empty)
                {
                    return false;
                }
            }
            {
                ConstraintValidationResult constraint = getConstraintType(k,i,j);
                if(constraint.st_final != constraint.st)
                {
                    constraint.constraint_st->setType(constraint.st_final);
                    // enforce symmetry
                    QualitativeTimePointConstraint::Ptr jk = getBidirectionalConstraint(j,k);

                    if(jk->getType() == QualitativeTimePointConstraint::Empty)
                    {
                        return false;
                    } else {
                        edges.insert(VertexPair(i,k));
                    }
                } else if(constraint.st_final == QualitativeTimePointConstraint::Empty)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

bool QualitativeTemporalConstraintNetwork::incrementalPathConsistency()
{
    std::vector<Vertex::Ptr> vertices = getGraph()->getAllVertices();
    LOG_DEBUG_S << "Incremental path consistency: updated constraints #" << mUpdatedConstraints.size();
    while(!mUpdatedConstraints.empty())
    {
        VertexPair pair = *mUpdatedConstraints.begin();
        mUpdatedConstraints.erase(mUpdatedConstraints.begin());

        const Vertex::Ptr& i = pair.first;
        const Vertex::Ptr& j = pair.second;

        std::vector<Vertex::Ptr>::const_iterator cit = vertices.begin();

        // Handle corner case for small
        size_t numberOfVertices = vertices.size();
        if(numberOfVertices == 2)
        {
            try {
                getBidirectionalConstraintType(vertices[0], vertices[1]);
            } catch(const std::runtime_error& e)
            {
                // invalid constrains
                return false;
            }
        }

        for(; cit != vertices.end(); ++cit)
        {
            const Vertex::Ptr& k = *cit;

            if(k == i || k == j )
            {
                // not a triangle thus continue
                continue;
            }

            VertexPair ik = revise(i, k, composition(i,j,k));
            if(ik != VertexPair())
            {
                mUpdatedConstraints.insert(ik);
            }

            VertexPair jk = revise(j, k, composition(j,i,k));
            if(jk != VertexPair())
            {
                mUpdatedConstraints.insert(jk);
            }
        }
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
    QualitativeTimePointConstraint::Type ij = QualitativeTemporalConstraintNetwork::getDirectionalConstraintType(i,j);
    QualitativeTimePointConstraint::Type intersectionType = QualitativeTimePointConstraint::getIntersection(ij, pathConstraintType);

    LOG_DEBUG_S << "Revise: (i: " << i->getLabel() << ", j: " << j->getLabel() << "): " << QualitativeTimePointConstraint::TypeTxt[pathConstraintType] << ":  and bidirectional: " << QualitativeTimePointConstraint::TypeTxt[ij] << " --> "
       << QualitativeTimePointConstraint::TypeTxt[intersectionType];

    if(intersectionType == QualitativeTimePointConstraint::Empty)
    {
        throw std::runtime_error("QualitativeTimePointConstraint::revise: inconsistent constraint: '" + i->toString() + "' - '" + j->toString());
    }

    if(intersectionType == ij)
    {
        return VertexPair();
    }

    setConstraintType(i,j, intersectionType);
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
    base::Time start = base::Time::now();
    std::vector<Vertex::Ptr> triangle = vertexTriangle;

    int i = 0;
    int j = 2;
    int k = 1;

    try {
        //LOG_DEBUG_S << "Checking triangle: " << triangle[i]->toString() << " -- " << triangle[j]->toString() <<
        //    " -- " << triangle[k]->toString();
        QualitativeTimePointConstraint::Type ij = getBidirectionalConstraintType(triangle[i], triangle[j]);
        QualitativeTimePointConstraint::Type ik = getBidirectionalConstraintType(triangle[i], triangle[k]);
        QualitativeTimePointConstraint::Type kj = getBidirectionalConstraintType(triangle[k], triangle[j]);

        QualitativeTimePointConstraint::Type ij_composition = QualitativeTimePointConstraint::getComposition(ik,kj);

        QualitativeTimePointConstraint::Type compositionType = QualitativeTimePointConstraint::getIntersection(ij, ij_composition);
        if(compositionType == QualitativeTimePointConstraint::Empty)
        {
            return false;
        }
    } catch(const std::runtime_error& e)
    {
        LOG_INFO_S << e.what();
        return false;
    }

    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent(const graph_analysis::Vertex::Ptr& v0, const graph_analysis::Vertex::Ptr& v1)
{
    try {
        QualitativeTemporalConstraintNetwork::getBidirectionalConstraintType(v0, v1);
        return true;
    } catch(const std::runtime_error& e)
    {
        return false;
    }
}

QualitativeTimePointConstraint::Ptr QualitativeTemporalConstraintNetwork::getDirectionalConstraint(const graph_analysis::Vertex::Ptr& i, const graph_analysis::Vertex::Ptr& j)
{
    std::vector<Edge::Ptr> edgesIJ = getGraph()->getEdges(i, j);
    if(edgesIJ.empty())
    {
        QualitativeTimePointConstraint::Ptr constraint = addQualitativeConstraint(dynamic_pointer_cast<TimePoint>(i), dynamic_pointer_cast<TimePoint>(j), QualitativeTimePointConstraint::Universal);
        return constraint;
    } else {
        //assert(edgesIJ.size() == 1);
        QualitativeTimePointConstraint::Ptr constraint = dynamic_pointer_cast<QualitativeTimePointConstraint>(edgesIJ[0]);
        if(!constraint)
        {
            throw std::invalid_argument("templ::solvers::temporal::QualitativeTemporalConstraintNetwork::getDirectionalConstraintType: "
                    " edge is not a QualitativeTimePointConstraint, but " + edgesIJ[0]->getClassName());
        }
        return constraint;
    }
}

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getDirectionalConstraintType(const graph_analysis::Vertex::Ptr& i, const graph_analysis::Vertex::Ptr& j)
{
    QualitativeTimePointConstraint::Ptr constraint = getDirectionalConstraint(i,j);
    return constraint->getType();
}

QualitativeTimePointConstraint::Ptr QualitativeTemporalConstraintNetwork::getBidirectionalConstraint(const graph_analysis::Vertex::Ptr& i, const graph_analysis::Vertex::Ptr& j)
{
    QualitativeTimePointConstraint::Ptr ij = getDirectionalConstraint(i,j);
    QualitativeTimePointConstraint::Ptr ji = getDirectionalConstraint(j,i);

    QualitativeTimePointConstraint::Type result = QualitativeTimePointConstraint::getIntersection(ij->getType(), QualitativeTimePointConstraint::getSymmetric(ji->getType()));

    ij->setType(result);
    ji->setType(QualitativeTimePointConstraint::getSymmetric(result));
    return ij;
}

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getBidirectionalConstraintType(const graph_analysis::Vertex::Ptr& i, const graph_analysis::Vertex::Ptr& j)
{
    if(i == j)
    {
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

   LOG_DEBUG_S << "Checking bidirectional type for: " << i->toString() << " -- " << j->toString();
    QualitativeTimePointConstraint::Type intersectionType = QualitativeTimePointConstraint::getIntersection(constraintTypeIJ, constraintTypeJI_sym);

   LOG_DEBUG_S << "Intersection of IJ (" << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << ")"
        << " and JI' (symmetric IJ) (" << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI_sym] << ")"
        << " --> " << QualitativeTimePointConstraint::TypeTxt[intersectionType];

    return intersectionType;
}

void QualitativeTemporalConstraintNetwork::setConstraintType(const Vertex::Ptr& i, const Vertex::Ptr& j, QualitativeTimePointConstraint::Type type)
{
    std::vector<Edge::Ptr> edges = getGraph()->getEdges(i,j);
    if(edges.empty())
    {
        addQualitativeConstraint(dynamic_pointer_cast<TimePoint>(i), dynamic_pointer_cast<TimePoint>(j), type);
    } else {
        QualitativeTimePointConstraint::Ptr constraint = dynamic_pointer_cast<QualitativeTimePointConstraint>(edges[0]);
        constraint->setType(type);
    }
}

QualitativeTemporalConstraintNetwork::ConstraintValidationResult QualitativeTemporalConstraintNetwork::getConstraintType(const Vertex::Ptr& s, const Vertex::Ptr& o, const Vertex::Ptr& t)
{
    ConstraintValidationResult r;
    r.constraint_st = getBidirectionalConstraint(s,t);
    r.st = r.constraint_st->getType();

    r.so = getBidirectionalConstraint(s,o)->getType();
    r.ot = getBidirectionalConstraint(o,t)->getType();

    r.st_composition = QualitativeTimePointConstraint::getComposition(r.so,r.ot);
    r.st_final = QualitativeTimePointConstraint::getIntersection(r.st, r.st_composition);
    return r;
}

void QualitativeTemporalConstraintNetwork::setConsistentNetwork(const graph_analysis::BaseGraph::Ptr& baseGraph)
{
    mUpdatedConstraints.clear();
    TemporalConstraintNetwork::setConsistentNetwork(baseGraph);
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
