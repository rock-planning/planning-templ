#include "SpaceTime.hpp"
#include <sstream>

namespace templ
{

SpaceTime::Network::tuple_t::Ptr SpaceTime::msHorizonStartTuple;
SpaceTime::Network::tuple_t::Ptr SpaceTime::msHorizonEndTuple;

std::string SpaceTime::toString(const Point& stp, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');

    assert(stp.first);
    ss << hspace;
    if(stp.first)
    {
        ss << stp.first->toString();
    } else {
        ss << " -- " << "unknown location";
    }

    if(stp.second)
    {
        ss << " -- " << stp.second->toString();
    } else {
        ss << " -- " << "unknown timepoint";
    }
    return ss.str();
}

std::string SpaceTime::toString(const Timeline& timeline, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "TIMELINE --" << std::endl;
    Timeline::const_iterator cit = timeline.begin();
    for(; cit != timeline.end(); ++cit)
    {
        const Point& stp = *cit;
        ss << toString(stp, indent + 4) << std::endl;
    }
    return ss.str();
}

std::string SpaceTime::toString(const Timelines& timelines, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << "TIMELINES --" << std::endl;

    Timelines::const_iterator cit = timelines.begin();
    for(; cit != timelines.end(); ++cit)
    {
        const Role& role = cit->first;
        const Timeline& timeline = cit->second;

        ss << hspace << role.toString() << std::endl;
        ss << toString(timeline, indent + 4);
    }
    return ss.str();
}

SpaceTime::Network SpaceTime::toNetwork(const symbols::constants::Location::PtrList& locations,
        const templ::solvers::temporal::point_algebra::TimePoint::PtrList& timepoints,
        const Timelines& timelines)
{
    Network network(locations, timepoints);
    Timelines::const_iterator cit = timelines.begin();
    for(; cit != timelines.end(); ++cit)
    {
        const Role& role = cit-> first;
        const Timeline& timeline = cit->second;

        Timeline::const_iterator tit = timeline.begin();
        for(; tit != timeline.end(); ++tit)
        {
            const Point& stp = *tit;
            const symbols::constants::Location::Ptr& location = stp.first;
            const solvers::temporal::point_algebra::TimePoint::Ptr& timepoint = stp.second;

            Network::tuple_t::Ptr roleInfo = network.tupleByKeys(location, timepoint);
            roleInfo->addRole(role);
        }
    }
    return network;
}

SpaceTime::Network::timepoint_t SpaceTime::getHorizonStart()
{
    // point_algebra::Timepoint
    return SpaceTime::Network::timepoint_t::element_type::create("__start__");
}

SpaceTime::Network::timepoint_t SpaceTime::getHorizonEnd()
{
    // point_algebra::Timepoint
    return SpaceTime::Network::timepoint_t::element_type::create("__end__");
}

SpaceTime::Network::value_t SpaceTime::getDepot()
{
    // Location
   return SpaceTime::Network::value_t::element_type::create("depot");
}

SpaceTime::Network::tuple_t::Ptr SpaceTime::getHorizonStartTuple()
{
    if(!msHorizonStartTuple)
    {
        msHorizonStartTuple = make_shared<SpaceTime::Network::tuple_t>(getDepot(), getHorizonStart());
    }
    return msHorizonStartTuple;
}

SpaceTime::Network::tuple_t::Ptr SpaceTime::getHorizonEndTuple()
{
    if(!msHorizonEndTuple)
    {
        msHorizonEndTuple = make_shared<SpaceTime::Network::tuple_t>(getDepot(), getHorizonEnd());
    }
    return msHorizonEndTuple;
}

void SpaceTime::injectVirtualStartAndEnd(SpaceTime::Network& network)
{
    using namespace graph_analysis;
    const BaseGraph::Ptr& graph = network.getGraph();
    VertexIterator::Ptr vertexIt = graph->getVertexIterator();
    while(vertexIt->next())
    {
        Vertex::Ptr vertex = vertexIt->current();
        bool isStart = false;
        bool isEnd = false;
        if( graph->getInEdges(vertex).empty() )
        {
            isStart = true;
        } else if(graph->getOutEdges(vertex).empty())
        {
            isEnd = true;
        } else {
            continue;
        }

        SpaceTime::Network::edge_t::Ptr edge = make_shared<SpaceTime::Network::edge_t>();
        edge->setWeight(std::numeric_limits<double>::max());
        if(isStart)
        {
            edge->setSourceVertex(getHorizonStartTuple());
            edge->setTargetVertex(vertex);
        } else if(isEnd)
        {
            edge->setSourceVertex(vertex);
            edge->setTargetVertex(getHorizonEndTuple());
        }
        graph->addEdge(edge);
    }
}

} // end namespace templ
