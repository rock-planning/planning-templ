#ifndef TEMPL_TEMPORALLY_EXPANDED_NETWORK_HPP
#define TEMPL_TEMPORALLY_EXPANDED_NETWORK_HPP

#include <limits>
#include <vector>
#include <stdexcept>
#include <graph_analysis/BaseGraph.hpp>
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
template<typename D0, typename D1 = templ::solvers::temporal::point_algebra::TimePoint::Ptr, typename TUPLE = Tuple<D0,D1> >
class TemporallyExpandedNetwork
{

public:
    typedef D0 value_t;
    typedef D1 timepoint_t;

    typedef TUPLE tuple_t;

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
    TupleMap mTupleMap;

public:
    TemporallyExpandedNetwork(const ValueList& values, const TimePointList& timepoints, const graph_analysis::Edge::Ptr& locationTransitionEdge = graph_analysis::Edge::Ptr())
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
            graph_analysis::WeightedEdge::Ptr weightedEdge(new graph_analysis::WeightedEdge());
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

    const graph_analysis::BaseGraph::Ptr& getGraph() const { return mpGraph; }

    void addTuple(const D0& value, const D1& timepoint, const typename tuple_t::Ptr& tuple)
    {
        mTupleMap[ ValueTimePair(value, timepoint) ] = tuple;
    }

    /**
     * Retrieve a tuple (actually a graph vertex) by the given key tuple
     */
    typename tuple_t::Ptr tupleByKeys(const D0& value, const D1& timepoint)
    {
        typename TupleMap::const_iterator cit = mTupleMap.find( ValueTimePair(value,timepoint) );
        if(cit != mTupleMap.end())
        {
            return cit->second;
        }

        throw std::invalid_argument("TemporallyExpandedNetwork::tupleByKeys: key does not exist");
    }

};

} // end namespace templ
#endif // TEMPL_TEMPORALLY_EXPANDED_NETWORK_HPP
