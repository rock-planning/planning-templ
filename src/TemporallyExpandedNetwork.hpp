#ifndef TEMPL_TEMPORALLY_EXPANDED_NETWORK_HPP
#define TEMPL_TEMPORALLY_EXPANDED_NETWORK_HPP

#include <limits>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <graph_analysis/BaseGraph.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/io/GraphvizWriter.hpp>
#include <graph_analysis/io/GraphvizGridStyle.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include "solvers/temporal/point_algebra/TimePoint.hpp"
#include "Tuple.hpp"

namespace templ {

/**
 * \class TemporallyExpandedNetwork
 * \tparam D0 First dimension, e.g., a constant such as Location
 * \tparam D1 Second dimension, which is Timepoint by default
 * \tparam TUPLE Additional information that can be stored in a map and is
 * accessible throught the key formed by std::pair<D0,D1>
 * \brief The temporally expanded network connects all tuples of the first
 * dimension (D0) if they belong to the same value of the second dimension D1
 * and assigns an infinite weight on this edge
 */
template<typename D0, typename D1 = templ::solvers::temporal::point_algebra::TimePoint::Ptr, typename TUPLE = Tuple<D0,D1>, typename EDGE_TYPE = graph_analysis::WeightedEdge >
class TemporallyExpandedNetwork
{

public:
    typedef D0 value_t;
    typedef D1 timepoint_t;

    typedef TUPLE tuple_t;
    typedef EDGE_TYPE edge_t;

    typedef std::vector<D0> ValueList;
    typedef std::vector<D1> TimePointList;

protected:
    ValueList mValues;
    TimePointList mTimepoints;
    graph_analysis::Edge::Ptr mpLocalTransitionEdge;
    graph_analysis::BaseGraph::Ptr mpGraph;

    typedef std::pair<value_t, timepoint_t> ValueTimePair;
    /// map allows to resolve from key-value --> tuple Ptr
    typedef std::map< ValueTimePair, typename tuple_t::Ptr > TupleMap;
    typedef typename tuple_t::PtrList TuplePtrList;
    TupleMap mTupleMap;

public:
    TemporallyExpandedNetwork()
        : mpGraph( graph_analysis::BaseGraph::getInstance() )
    {}

    TemporallyExpandedNetwork(const TemporallyExpandedNetwork& other)
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
    TemporallyExpandedNetwork(const ValueList& values,
            const TimePointList& timepoints,
            const graph_analysis::Edge::Ptr& locationTransitionEdge = graph_analysis::Edge::Ptr())
        : mValues(values)
        , mTimepoints(timepoints)
        , mpLocalTransitionEdge(locationTransitionEdge)
    {
        if(mValues.empty())
        {
            throw std::invalid_argument("templ::TemporallyExpandedNetwork: cannot construct network"
                    " since value list is empty");
        }
        if(mTimepoints.empty())
        {
            throw std::invalid_argument("templ::TemporallyExpandedNetwork: cannot construct network"
                    " since timepoint list is empty");
        }
        if(!mpLocalTransitionEdge)
        {
            graph_analysis::WeightedEdge::Ptr weightedEdge(new edge_t());
            weightedEdge->setWeight(std::numeric_limits<graph_analysis::WeightedEdge::value_t>::max());
            mpLocalTransitionEdge = weightedEdge;
        }
        initialize();
    }

    virtual ~TemporallyExpandedNetwork() {}

    /**
     * Set the edge that has to be assigned for all transitions between same
     * types of the first dimension
     */
    void setLocalTransitionEdge(const graph_analysis::Edge::Ptr& edge) { mpLocalTransitionEdge = edge; }

    // Construction of the basic time-expanded network
    //
    // (t0,v0)    (t0,v1)
    //    |          |
    // (t1,v0)    (t1,v1)
    //
    void initialize()
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
    void reconstructTupleMap()
    {
        graph_analysis::VertexIterator::Ptr vertexIt = mpGraph->getVertexIterator();
        while(vertexIt->next())
        {
            typename tuple_t::Ptr currentTuple = dynamic_pointer_cast<tuple_t>( vertexIt->current() );
            mTupleMap[ ValueTimePair(currentTuple->first(), currentTuple->second()) ] = currentTuple;
        }
    }

    const graph_analysis::BaseGraph::Ptr& getGraph() const { return mpGraph; }

    /**
     * Return the list of values, e.g. for the SpaceTime::Network that will mean
     * the locations
     */
    const ValueList& getValues() const { return mValues; }

    /**
     * Return the list of timepoints
     */
    const TimePointList& getTimepoints() const { return mTimepoints; }

    void addTuple(const D0& value, const D1& timepoint, const typename tuple_t::Ptr& tuple)
    {
        mTupleMap[ ValueTimePair(value, timepoint) ] = tuple;
    }

    /**
     * Retrieve a tuple (actually a graph vertex) by the given key tuple
     * \param value
     * \param timepoint
     * \return tuple
     */
    typename tuple_t::Ptr tupleByKeys(const D0& value, const D1& timepoint) const
    {
        typename TupleMap::const_iterator cit = mTupleMap.find( ValueTimePair(value,timepoint) );
        if(cit != mTupleMap.end())
        {
            return cit->second;
        }

        throw std::invalid_argument("TemporallyExpandedNetwork::tupleByKeys: key does not exist");
    }

    void save(const std::string& filename, const std::string& type = "dot") const
    {
        using namespace graph_analysis::io;
        if(type == "dot")
        {
            GraphvizWriter gvWriter("dot","canon");
            GraphvizGridStyle::Ptr style(new GraphvizGridStyle(
                        mValues.size(),
                        mTimepoints.size(),
                        bind(&TemporallyExpandedNetwork<D0,D1,TUPLE,EDGE_TYPE>::getRow,this, placeholder::_1),
                        bind(&TemporallyExpandedNetwork<D0,D1,TUPLE,EDGE_TYPE>::getColumn,this, placeholder::_1)
                        ));
            style->setColumnScalingFactor(5.0);
            style->setRowScalingFactor(5.0);

            gvWriter.setStyle(style);
            assert(mpGraph);
            gvWriter.write(filename, mpGraph);
        } else if(type == "gexf")
        {
            assert(mpGraph);
            graph_analysis::io::GraphIO::write(filename,mpGraph, graph_analysis::representation::GEXF);
        }
    }

    static TemporallyExpandedNetwork<D0,D1,TUPLE, EDGE_TYPE> fromFile(const std::string& filename, const std::vector<D0>& values, const std::vector<D1>& timepoints)
    {
        TemporallyExpandedNetwork network;
        graph_analysis::io::GraphIO::read(filename, network.mpGraph);

        network.mValues = values;
        network.mTimepoints = timepoints;
        network.reconstructTupleMap();

        return network;
    }

    const ValueTimePair& getValueTimePair(const typename tuple_t::Ptr& searchTuple) const
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
        throw std::invalid_argument("templ::TemporallyExpandedNetwork::getValueTimePair: could not find provided tuple in network");
    }

    const D0& getValue(const typename tuple_t::Ptr& tuple) const
    {
        return getValueTimePair(tuple).first;
    }

    const D1& getTimepoint(const typename tuple_t::Ptr& tuple) const
    {
        return getValueTimePair(tuple).second;
    }

    size_t getColumn(const graph_analysis::Vertex::Ptr& vertex) const
    {
        typename tuple_t::Ptr tuple = dynamic_pointer_cast<tuple_t>(vertex);
        const D0& value = getValue(tuple);

        typename ValueList::const_iterator cit = std::find_if(mValues.begin(), mValues.end(), [value](const D0& other)
                {
                    return value == other;
                });
        return std::distance(mValues.begin(), cit);
    }

    size_t getRow(const graph_analysis::Vertex::Ptr& vertex) const
    {
        typename tuple_t::Ptr tuple = dynamic_pointer_cast<tuple_t>(vertex);
        const D1& timepoint = getTimepoint(tuple);

        typename TimePointList::const_iterator cit = std::find_if(mTimepoints.begin(), mTimepoints.end(), [timepoint](const D1& other)
                {
                    return timepoint == other;
                });
        return std::distance(mTimepoints.begin(), cit);
    }

    /**
     * Get all timepoints that are part of a given interval
     */
    TimePointList getTimepoints(const timepoint_t& t_start, const timepoint_t& t_end) const
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
            throw std::invalid_argument("templ::TemporallyExpandedNetwork::getTimepoints: failed to identify timepoints"
                    " for interval from '" << t_start->toString() << " to '" << t_end->toString());
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
    TuplePtrList getTuples(const timepoint_t& t_start, const timepoint_t& t_end, const value_t& value) const
    {
        TuplePtrList tuples;
        TimePointList timepoints = getTimepoints(t_start, t_end);
        for(const timepoint_t& t : timepoints)
        {
            tuples.push_back( tupleByKeys(value,t) );
        }
        return tuples;
    }
};

} // end namespace templ
#endif // TEMPL_TEMPORALLY_EXPANDED_NETWORK_HPP
