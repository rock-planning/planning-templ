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

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getConstraint(TimePoint::Ptr t1, TimePoint::Ptr t2)
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
//    typedef std::vector<Edge::Ptr> Path;
//    typedef std::vector<Vertex::Ptr> Neighbours;
//    std::vector< std::vector<Neighbours> > neighbours;
//    std::vector< std::vector<Path> > paths;
//
//    LOG_WARN_S << "Search: 0";
//    SubGraph::Ptr subgraph = graph_analysis::BaseGraph::getSubGraph(getGraph());
//    LOG_WARN_S << "Search: 1";
//
//    Path path;
//    std::vector<Vertex::Ptr> vertices;
//    // No path with length 0
//    paths.push_back( std::vector<Path>() );
//    vertices.push_back( t1 );
//    LOG_WARN_S << "Search: 2";
//    for(uint8_t i = 1; i <= maximumPathDepth; ++i)
//    {
//        std::vector<Path> pathsWithDepthI;
//        std::vector<Vertex::Ptr> nextVertexList;
//        std::vector<Vertex::Ptr>::const_iterator nit = vertices.begin();
//        LOG_WARN_S << "Search: 3";
//        for(; nit != vertices.end(); ++nit)
//        {
//            Vertex::Ptr vertex = *nit;
//            graph_analysis::EdgeIterator::Ptr edgeIt = subgraph->getEdgeIterator(vertex);
//            while(edgeIt->next())
//            {
//                Path currentPath = path;
//
//                Edge::Ptr edge = edgeIt->current();
//                // path will be constructed inverse, so last element at position 0
//                currentPath.insert(currentPath.begin(), edge);
//                pathsWithDepthI.push_back(currentPath);
//
//                subgraph->disable(edge);
//                nextVertexList.push_back(edge->getTargetVertex());
//            }
//            vertices = nextVertexList;
//        }
//        LOG_WARN_S << "Pushback " << (int) i << " --> path size: " << pathsWithDepthI.size();
//        paths.push_back(pathsWithDepthI);
//    }
//
//    LOG_WARN_S << "Search complete: number of paths: " << paths.size();
//    std::vector<Path> validPaths;
//    // Keep only paths that lead to t2
//    for(uint8_t d = 1; d <= maximumPathDepth; ++d)
//    {
//        LOG_WARN_S << "Get path at: " << (int) d;
//        uint16_t numberOfPaths = paths[d].size();
//        LOG_WARN_S << "Availabe: " << numberOfPaths;
//        for(uint8_t n = 0; n < numberOfPaths; ++n)
//        {
//            Path path = paths[d][n];
//            Edge::Ptr edge = path[0];
//            if( boost::dynamic_pointer_cast<TimePoint>(edge)->equals(t2))
//            {
//                validPaths.push_back(path);
//            }
//        }
//    }
//    LOG_WARN_S << "Valid path completed: size " << validPaths.size();
//
//    std::vector<QualitativeTimePointConstraint::Type> constraints;
//    // All paths should now represent a consistent constraint
//    // i.e. translate path into composition
//    std::vector<Path>::const_iterator cit = validPaths.begin();
//    for(; cit != validPaths.end(); ++cit)
//    {
//        LOG_WARN_S << "Path ----";
//        Path::const_iterator pit = cit->begin();
//        std::vector<QualitativeTimePointConstraint::Type> typeList;
//        for(; pit != cit->end(); ++pit)
//        {
//            Edge::Ptr edge = *pit;
//            QualitativeTimePointConstraint::Ptr constraint = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edge);
//            typeList.push_back(constraint->getType());
//            LOG_WARN_S << "    " << QualitativeTimePointConstraint::TypeTxt[constraint->getType()];
//        }
//        QualitativeTimePointConstraint::Type type = QualitativeTimePointConstraint::getComposition(typeList);
//
//        LOG_WARN_S << " composition -->" << QualitativeTimePointConstraint::TypeTxt[type];
//        constraints.push_back(type);
//    }
//
//    return QualitativeTimePointConstraint::getComposition(constraints);
//
//
//    //graph_analysis::lemon::DirectedGraph* digraph = dynamic_cast<graph_analysis::lemon::DirectedGraph*>(getGraph().get());
//    //assert(digraph);
//    //::lemon::ListDigraph::Node startNode = digraph->getNode(t1);
//    //::lemon::ListDigraph::Node endNode = digraph->getNode(t2);
//
//    //::lemon::Bfs< ::lemon::ListDigraph> bfs(digraph->raw());
//    //for(uint8_t i = 0; i < maximumPathDepth; ++i)
//    //{
//    //    bfs.init();
//    //    bfs.addSource(startNode);
//    //    typedef std::vector<::lemon::ListDigraph::Arc> Path;
//    //    std::vector< std::vector<Path> > paths;
//    //    while(!bfs.emptyQueue())
//    //    {
//    //        ::lemon::ListDigraph::Node node = bfs.processNextNode();
//    //        if(node == endNode)
//    //        {
//    //        }
//    //    }
//    //}
//    ////::lemon::Bfs<::lemon::ListDigraph> bfs(digraph->raw());
//    ////bfs.run(digraph->getNode(t1), digraph->getNode(t2));
//    ////std::map<::lemon::ListDigraph::Arc, double> lengthmap;
//    //typedef ::lemon::concepts::ReadMap< ::lemon::ListDigraph::Arc, int> LengthMap;
//    ////typedef ::lemon::concepts::ReadWriteMap<::lemon::ListDigraph::Node,int> DistMap;
//    //typedef int DistMap;
//    ////typedef ::lemon::Dijkstra< ::lemon::ListDigraph, LengthMap>::DistMap DistMap;
//
//    //LengthMap lengthmap;
//    ////DistMap distmap;
//    //::lemon::Path< ::lemon::ListDigraph> path;
//    ////bool reached = ::lemon::dijkstra(digraph->raw(), lengthmap).path(path).dist(distmap).run(digraph->getNode(t1), digraph->getNode(t2));
//    //::lemon::Dijkstra< ::lemon::ListDigraph> dijkstra(digraph->raw(), lengthmap);
//    //dijkstra.init();
//    //dijkstra.addSource(startNode);
//    //dijkstra.start(endNode);
//
//    ////bool reached = ::lemon::dijkstra(digraph->raw(), lengthmap).path(path).run(digraph->getNode(t1), digraph->getNode(t2));
//
//    //std::vector<QualitativeTimePointConstraint::Ptr> constraintList;
//    //if(!reached)
//    //{
//    //    return constraintList;
//    //} else {
//    //    int pathLength = path.length();
//    //    for(int i = 0; i < pathLength; ++i)
//    //    {
//    //        ::lemon::ListDigraph::Arc arc = path.nth(i);
//    //        QualitativeTimePointConstraint::Ptr qtpc = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>( digraph->getEdge(arc) );
//    //        if(qtpc)
//    //        {
//    //            constraintList.push_back(qtpc);
//    //        }
//    //    }
//    //}
//
//    //return constraintList;
//    //bool reached = dijkstra(g, length).path(p).dist(d).run( digraph->getNode(t1), digraph->getNode(t2));
//
//  //  if(edges.empty())
//  //  {
//
//  //  }
//}

Constraint::Ptr QualitativeTemporalConstraintNetwork::addQualitativeConstraint(TimePoint::Ptr t1, TimePoint::Ptr t2, QualitativeTimePointConstraint::Type constraintType)
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

bool QualitativeTemporalConstraintNetwork::isConsistent(graph_analysis::Vertex::Ptr v0, graph_analysis::Vertex::Ptr v1)
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

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getDirectionalConstraintType(graph_analysis::Vertex::Ptr i, graph_analysis::Vertex::Ptr j) const
{
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

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getBidirectionalConstraintType(graph_analysis::Vertex::Ptr i, graph_analysis::Vertex::Ptr j) const
{
    QualitativeTimePointConstraint::Type constraintTypeIJ = getDirectionalConstraintType(i,j);
    QualitativeTimePointConstraint::Type constraintTypeJI = QualitativeTimePointConstraint::getSymmetric( getDirectionalConstraintType(j,i) );

    if(!QualitativeTimePointConstraint::isConsistent(constraintTypeIJ, constraintTypeJI))
    {
        LOG_DEBUG_S << "IJ symmetry assumption does not hold" <<
            " ij: " << QualitativeTimePointConstraint::TypeTxt[constraintTypeIJ] << " vs. ji " << QualitativeTimePointConstraint::TypeTxt[constraintTypeJI];
        throw std::runtime_error("QualitativeTimePointConstraint::getBidirectionalConstraintType: IJ - JI with inconsistent contraints");
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
