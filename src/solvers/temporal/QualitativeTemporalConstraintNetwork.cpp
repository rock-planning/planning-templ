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

QualitativeTimePointConstraint::Type QualitativeTemporalConstraintNetwork::getConstraint(TimePoint::Ptr t1, TimePoint::Ptr t2, uint8_t maximumPathDepth)
{
    typedef std::vector<Edge::Ptr> Path;
    typedef std::vector<Vertex::Ptr> Neighbours;
    std::vector< std::vector<Neighbours> > neighbours;
    std::vector< std::vector<Path> > paths;

    LOG_WARN_S << "Search: 0";
    SubGraph::Ptr subgraph = graph_analysis::BaseGraph::getSubGraph(getGraph());
    LOG_WARN_S << "Search: 1";

    Path path;
    std::vector<Vertex::Ptr> vertices;
    // No path with length 0
    paths.push_back( std::vector<Path>() );
    vertices.push_back( t1 );
    LOG_WARN_S << "Search: 2";
    for(uint8_t i = 1; i <= maximumPathDepth; ++i)
    {
        std::vector<Path> pathsWithDepthI;
        std::vector<Vertex::Ptr> nextVertexList;
        std::vector<Vertex::Ptr>::const_iterator nit = vertices.begin();
        LOG_WARN_S << "Search: 3";
        for(; nit != vertices.end(); ++nit)
        {
            Vertex::Ptr vertex = *nit;
            graph_analysis::EdgeIterator::Ptr edgeIt = subgraph->getEdgeIterator(vertex);
            while(edgeIt->next())
            {
                Path currentPath = path;

                Edge::Ptr edge = edgeIt->current();
                // path will be constructed inverse, so last element at position 0
                currentPath.insert(currentPath.begin(), edge);
                pathsWithDepthI.push_back(currentPath);

                subgraph->disable(edge);
                nextVertexList.push_back(edge->getTargetVertex());
            }
            vertices = nextVertexList;
        }
        LOG_WARN_S << "Pushback " << (int) i << " --> path size: " << pathsWithDepthI.size();
        paths.push_back(pathsWithDepthI);
    }

    LOG_WARN_S << "Search complete: number of paths: " << paths.size();
    std::vector<Path> validPaths;
    // Keep only paths that lead to t2
    for(uint8_t d = 1; d <= maximumPathDepth; ++d)
    {
        LOG_WARN_S << "Get path at: " << (int) d;
        uint16_t numberOfPaths = paths[d].size();
        LOG_WARN_S << "Availabe: " << numberOfPaths;
        for(uint8_t n = 0; n < numberOfPaths; ++n)
        {
            Path path = paths[d][n];
            Edge::Ptr edge = path[0];
            if( boost::dynamic_pointer_cast<TimePoint>(edge)->equals(t2))
            {
                validPaths.push_back(path);
            }
        }
    }
    LOG_WARN_S << "Valid path completed: size " << validPaths.size();

    std::vector<QualitativeTimePointConstraint::Type> constraints;
    // All paths should now represent a consistent constraint
    // i.e. translate path into composition
    std::vector<Path>::const_iterator cit = validPaths.begin();
    for(; cit != validPaths.end(); ++cit)
    {
        LOG_WARN_S << "Path ----";
        Path::const_iterator pit = cit->begin();
        std::vector<QualitativeTimePointConstraint::Type> typeList;
        for(; pit != cit->end(); ++pit)
        {
            Edge::Ptr edge = *pit;
            QualitativeTimePointConstraint::Ptr constraint = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(edge);
            typeList.push_back(constraint->getType());
            LOG_WARN_S << "    " << QualitativeTimePointConstraint::TypeTxt[constraint->getType()];
        }
        QualitativeTimePointConstraint::Type type = QualitativeTimePointConstraint::getComposition(typeList);

        LOG_WARN_S << " composition -->" << QualitativeTimePointConstraint::TypeTxt[type];
        constraints.push_back(type);
    }

    return QualitativeTimePointConstraint::getComposition(constraints);
    

    //graph_analysis::lemon::DirectedGraph* digraph = dynamic_cast<graph_analysis::lemon::DirectedGraph*>(getGraph().get());
    //assert(digraph);
    //::lemon::ListDigraph::Node startNode = digraph->getNode(t1);
    //::lemon::ListDigraph::Node endNode = digraph->getNode(t2);

    //::lemon::Bfs< ::lemon::ListDigraph> bfs(digraph->raw());
    //for(uint8_t i = 0; i < maximumPathDepth; ++i)
    //{
    //    bfs.init();
    //    bfs.addSource(startNode);
    //    typedef std::vector<::lemon::ListDigraph::Arc> Path;
    //    std::vector< std::vector<Path> > paths;
    //    while(!bfs.emptyQueue())
    //    {
    //        ::lemon::ListDigraph::Node node = bfs.processNextNode();
    //        if(node == endNode)
    //        {
    //        }
    //    }
    //}
    ////::lemon::Bfs<::lemon::ListDigraph> bfs(digraph->raw());
    ////bfs.run(digraph->getNode(t1), digraph->getNode(t2));
    ////std::map<::lemon::ListDigraph::Arc, double> lengthmap;
    //typedef ::lemon::concepts::ReadMap< ::lemon::ListDigraph::Arc, int> LengthMap;
    ////typedef ::lemon::concepts::ReadWriteMap<::lemon::ListDigraph::Node,int> DistMap;
    //typedef int DistMap;
    ////typedef ::lemon::Dijkstra< ::lemon::ListDigraph, LengthMap>::DistMap DistMap;

    //LengthMap lengthmap;
    ////DistMap distmap;
    //::lemon::Path< ::lemon::ListDigraph> path;
    ////bool reached = ::lemon::dijkstra(digraph->raw(), lengthmap).path(path).dist(distmap).run(digraph->getNode(t1), digraph->getNode(t2));
    //::lemon::Dijkstra< ::lemon::ListDigraph> dijkstra(digraph->raw(), lengthmap);
    //dijkstra.init();
    //dijkstra.addSource(startNode);
    //dijkstra.start(endNode);

    ////bool reached = ::lemon::dijkstra(digraph->raw(), lengthmap).path(path).run(digraph->getNode(t1), digraph->getNode(t2));

    //std::vector<QualitativeTimePointConstraint::Ptr> constraintList;
    //if(!reached)
    //{
    //    return constraintList;
    //} else {
    //    int pathLength = path.length();
    //    for(int i = 0; i < pathLength; ++i)
    //    {
    //        ::lemon::ListDigraph::Arc arc = path.nth(i);
    //        QualitativeTimePointConstraint::Ptr qtpc = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>( digraph->getEdge(arc) );
    //        if(qtpc)
    //        {
    //            constraintList.push_back(qtpc);
    //        }
    //    }
    //}

    //return constraintList;
    //bool reached = dijkstra(g, length).path(p).dist(d).run( digraph->getNode(t1), digraph->getNode(t2));

  //  if(edges.empty())
  //  {

  //  }
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

bool QualitativeTemporalConstraintNetwork::isConsistent(const std::vector<Edge::Ptr>& edges)
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
        if(!isConsistent(triangle))
        {
            return false;
        }

    } while(combination.next());

    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent(graph_analysis::Vertex::Ptr i, graph_analysis::Vertex::Ptr j)
{
    std::vector<Edge::Ptr> edgesIJ = getGraph()->getEdges(i, j);
    if (edgesIJ.size() > 1)
    {
        throw std::runtime_error("QualitativeTemporalConstraintNetwork::isConsistent: cannot proceed since assumption of only 1 directional constraint between two vertices does not hold");
    }

    // Universal constraint -- if there is no edge defined
    // IJ -- Constraint between node I and J
    QualitativeTimePointConstraint::Type constraintTypeIJ = QualitativeTimePointConstraint::Empty;
    QualitativeTimePointConstraint::Type constraintTypeJI = getSymmetricConstraint(getGraph(), j, i);
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
    return true;
}

bool QualitativeTemporalConstraintNetwork::isConsistent(const std::vector<Vertex::Ptr>& triangle)
{
    int i = 0;
    int j = 2;
    int k = 1;

    return isConsistent(triangle[i], triangle[j]) && isConsistent(triangle[i], triangle[k]) && isConsistent(triangle[k], triangle[j]);
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
