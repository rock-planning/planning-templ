#include "SpaceTimeNetwork.hpp"

namespace templ {

SpaceTimeNetwork::SpaceTimeNetwork()
    : mpGraph( graph_analysis::BaseGraph::getInstance() )
{}

SpaceTimeNetwork::SpaceTimeNetwork(const SpaceTimeNetwork& other)
    : mValues(other.mValues)
    , mTimepoints(other.mTimepoints)
    , mpLocalTransitionEdge(other.mpLocalTransitionEdge)
    , mTupleMap(other.mTupleMap)
{
    if(other.mpGraph)
    {
        mpGraph = other.mpGraph->copy();
    }
}

/**
 * Constructor for a temporally expanded network from a set of values and
 * timepoints
 * \param values List of symbols that should be temporally qualified
 * \param timepoints List of timepoints which are assumed to be sorted
 * according to time
 * \param locationTransitionEdge type of edge which is created for a
 * location transition edge
 */
SpaceTimeNetwork::SpaceTimeNetwork(const ValueList& values,
        const TimePointList& timepoints,
        const graph_analysis::Edge::Ptr& locationTransitionEdge)
    : mValues(values)
    , mTimepoints(timepoints)
    , mpLocalTransitionEdge(locationTransitionEdge)
{
    if(mValues.empty())
    {
        throw std::invalid_argument("templ::SpaceTimeNetwork: cannot construct network"
                " since value list is empty");
    }
    if(mTimepoints.empty())
    {
        throw std::invalid_argument("templ::SpaceTimeNetwork: cannot construct network"
                " since timepoint list is empty");
    }
    if(!mpLocalTransitionEdge)
    {
        graph_analysis::WeightedEdge::Ptr weightedEdge = make_shared<edge_t>();
        weightedEdge->setWeight(std::numeric_limits<graph_analysis::WeightedEdge::value_t>::max());
        mpLocalTransitionEdge = weightedEdge;
    }
    initialize();
}

SpaceTimeNetwork::~SpaceTimeNetwork()
{}

// Construction of the basic time-expanded network
//
// (t0,v0)    (t0,v1)
//    |          |
// (t1,v0)    (t1,v1)
//
void SpaceTimeNetwork::initialize()
{
    mpGraph = graph_analysis::BaseGraph::getInstance();

    typename ValueList::const_iterator lit = mValues.begin();
    for(; lit != mValues.end(); ++lit)
    {
        typename tuple_t::Ptr previousTuple;

        typename TimePointList::const_iterator tit = mTimepoints.begin();
        for(; tit != mTimepoints.end(); ++tit)
        {
            typename tuple_t::Ptr currentTuple(new tuple_t(*lit, *tit));
            mpGraph->addVertex(currentTuple);

            mTupleMap[ ValueTimePair(*lit, *tit) ] = currentTuple;

            if(previousTuple)
            {
                using namespace graph_analysis;

                Edge::Ptr edge = mpLocalTransitionEdge->clone();
                edge->setSourceVertex(previousTuple);
                edge->setTargetVertex(currentTuple);
                mpGraph->addEdge(edge);
            }
            previousTuple = currentTuple;
        }
    }
}

/**
 * Avoid the need for serialization of the tuple map, and reconstruct it
 * instead from the set of timepoints and fluent
 */
void SpaceTimeNetwork::reconstructTupleMap()
{
    graph_analysis::VertexIterator::Ptr vertexIt = mpGraph->getVertexIterator();
    while(vertexIt->next())
    {
        typename tuple_t::Ptr currentTuple = dynamic_pointer_cast<tuple_t>( vertexIt->current() );
        mTupleMap[ ValueTimePair(currentTuple->first(), currentTuple->second()) ] = currentTuple;
    }
}

void SpaceTimeNetwork::addTuple(const value_t& value,
        const timepoint_t& timepoint,
        const typename tuple_t::Ptr& tuple)
{
    mTupleMap[ ValueTimePair(value, timepoint) ] = tuple;
}

/**
 * Retrieve a tuple (actually a graph vertex) by the given key tuple
 * \param value
 * \param timepoint
 * \return tuple
 */
typename SpaceTimeNetwork::tuple_t::Ptr SpaceTimeNetwork::tupleByKeys(const value_t& value, const timepoint_t& timepoint) const
{
    typename TupleMap::const_iterator cit = mTupleMap.find( ValueTimePair(value,timepoint) );
    if(cit != mTupleMap.end())
    {
        return cit->second;
    }

    throw std::invalid_argument("SpaceTimeNetwork::tupleByKeys: key does not exist");
}

void SpaceTimeNetwork::save(const std::string& filename, const std::string&
        type) const
{

    graph_analysis::representation::Type rep;
    if(type.empty())
    {
        try {
            rep = graph_analysis::io::GraphIO::getTypeFromFilename(filename);
        } catch(...)
        {
            LOG_WARN_S << "Could not extract file type: using DOT format";
            rep = graph_analysis::representation::GRAPHVIZ;
        }
    } else {
        rep = graph_analysis::io::GraphIO::getTypeFromSuffix(type);
    }

    using namespace graph_analysis::io;
    assert(mpGraph);
    if(rep == graph_analysis::representation::GRAPHVIZ)
    {
        GraphvizWriter gvWriter("dot","canon");
        GraphvizGridStyle::Ptr style(new GraphvizGridStyle(
                    mValues.size(),
                    mTimepoints.size(),
                    bind(&SpaceTimeNetwork::getRow,this, placeholder::_1),
                    bind(&SpaceTimeNetwork::getColumn,this, placeholder::_1)
                    ));
        style->setColumnScalingFactor(5.0);
        style->setRowScalingFactor(5.0);

        gvWriter.setStyle(style);
        gvWriter.write(filename, mpGraph);
    } else {
        graph_analysis::io::GraphIO::write(filename,mpGraph,rep);
    }
}

SpaceTimeNetwork SpaceTimeNetwork::fromFile(const std::string& filename, const std::vector<value_t>& values, const std::vector<timepoint_t>& timepoints)
{
    SpaceTimeNetwork network;
    graph_analysis::io::GraphIO::read(filename, network.mpGraph);

    network.mValues = values;
    network.mTimepoints = timepoints;
    network.reconstructTupleMap();

    return network;
}

SpaceTimeNetwork SpaceTimeNetwork::fromFile(const std::string& filename)
{
    using namespace graph_analysis;
    BaseGraph::Ptr graph = BaseGraph::getInstance();
    graph_analysis::io::GraphIO::read(filename, graph);

    EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
    std::set<value_t> values;

    using namespace solvers::temporal;

    QualitativeTemporalConstraintNetwork qtcn;

    while(edgeIt->next())
    {
        const Edge::Ptr& edge = edgeIt->current();

        shared_ptr< tuple_t > fromTuple = dynamic_pointer_cast<
            tuple_t >(edge->getSourceVertex());
        shared_ptr< tuple_t > toTuple = dynamic_pointer_cast<
            tuple_t >(edge->getTargetVertex());

        values.insert(fromTuple->first());

        qtcn.addQualitativeConstraint( fromTuple->second(),
                toTuple->second(),
                point_algebra::QualitativeTimePointConstraint::Less);
    }

    std::vector<timepoint_t> timepoints = solvers::csp::TemporalConstraintNetworkBase::getSortedList(qtcn);
    std::vector<value_t> uniqueValues(values.begin(), values.end());

    return fromGraph(graph, uniqueValues, timepoints);
}

SpaceTimeNetwork SpaceTimeNetwork::fromGraph(const graph_analysis::BaseGraph::Ptr& graph, const std::vector<value_t>& values, const std::vector<timepoint_t>& timepoints)
{
    SpaceTimeNetwork network;
    network.mpGraph = graph;

    network.mValues = values;
    network.mTimepoints = timepoints;
    network.reconstructTupleMap();

    return network;
}

const SpaceTimeNetwork::ValueTimePair& SpaceTimeNetwork::getValueTimePair(const typename tuple_t::Ptr& searchTuple) const
{
    typename TupleMap::const_iterator cit = mTupleMap.begin();
    for(; cit != mTupleMap.end(); ++cit)
    {
        const ValueTimePair& valueTimePair = cit->first;
        typename tuple_t::Ptr tuple = cit->second;
        if(tuple ==  searchTuple)
        {
            return valueTimePair;
        }
    }
    throw std::invalid_argument("templ::SpaceTimeNetwork::getValueTimePair: could not find provided tuple in network");
}

const SpaceTimeNetwork::value_t& SpaceTimeNetwork::getValue(const typename tuple_t::Ptr& tuple) const
{
    return getValueTimePair(tuple).first;
}

const SpaceTimeNetwork::timepoint_t& SpaceTimeNetwork::getTimepoint(const typename tuple_t::Ptr& tuple) const
{
    return getValueTimePair(tuple).second;
}

size_t SpaceTimeNetwork::getColumn(const graph_analysis::Vertex::Ptr& vertex) const
{
    typename tuple_t::Ptr tuple = dynamic_pointer_cast<tuple_t>(vertex);
    const value_t& value = getValue(tuple);

    typename ValueList::const_iterator cit = std::find_if(mValues.begin(), mValues.end(), [value](const value_t& other)
            {
                return value == other;
            });
    return std::distance(mValues.begin(), cit);
}

size_t SpaceTimeNetwork::getRow(const graph_analysis::Vertex::Ptr& vertex) const
{
    typename tuple_t::Ptr tuple = dynamic_pointer_cast<tuple_t>(vertex);
    const timepoint_t& timepoint = getTimepoint(tuple);

    typename TimePointList::const_iterator cit = std::find_if(mTimepoints.begin(), mTimepoints.end(), [timepoint](const timepoint_t& other)
            {
                return timepoint == other;
            });
    return std::distance(mTimepoints.begin(), cit);
}

/**
 * Get all timepoints that are part of a given interval
 */
SpaceTimeNetwork::TimePointList SpaceTimeNetwork::getTimepoints(const timepoint_t& t_start, const timepoint_t& t_end) const
{
    TimePointList timepoints;
    bool startFound = false;
    bool endFound = false;
    for(const timepoint_t& t : mTimepoints)
    {
        if(t->equals(t_start))
        {
            startFound = true;
        }
        if(startFound)
        {
            timepoints.push_back(t);
        }
        if(t->equals(t_end))
        {
            endFound = true;
            break;
        }
    }
    if(!startFound && !endFound)
    {
        throw std::invalid_argument("templ::SpaceTimeNetwork::getTimepoints: failed to identify timepoints"
                " for interval from '" + t_start->toString() + " to '" + t_end->toString());
    }
    return timepoints;
}

/**
 * Get all tuples for a given interval and a given temporally
 * expanded value
 * \param t_start start timepoint of the interval
 * \param t_end end timepoint of the interval
 * \param value value (e.g. location symbol) which has a temporal expansion
 * \return list of tuples pointers
 */
SpaceTimeNetwork::TuplePtrList SpaceTimeNetwork::getTuples(const timepoint_t& t_start,
        const timepoint_t& t_end,
        const value_t& value) const
{
    TuplePtrList tuples;
    TimePointList timepoints = getTimepoints(t_start, t_end);
    for(const timepoint_t& t : timepoints)
    {
        try {
            tuples.push_back( tupleByKeys(value,t) );
        } catch(const std::exception& e)
        {
            // only the relevant and ones available are considered
            LOG_DEBUG_S << "Query with unregistered key: " << e.what();
        }
    }
    return tuples;
}

}
