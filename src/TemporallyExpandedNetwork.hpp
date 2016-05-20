#ifndef TEMPL_TEMPORALLY_EXPANDED_NETWORK_HPP
#define TEMPL_TEMPORALLY_EXPANDED_NETWORK_HPP

#include <limits>
#include <vector>
#include <stdexcept>
#include <graph_analysis/BaseGraph.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <templ/RoleInfoTuple.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>

namespace templ {

/**
 * \class TemporallyExpandedNetwork
 * \tparam D0
 * \tparam D1
 * \brief The temporally expanded network connects all tuples of the first
 * dimension (D0) if they belong to the same value of the second dimension D1
 * and assigns an infinite weight on this edge
 */
template<typename D0, typename D1 = templ::solvers::temporal::point_algebra::TimePoint>
class TemporallyExpandedNetwork
{

public:
    typedef D0 value_t;
    typedef D1 timepoint_t;

    typedef RoleInfoTuple<value_t, timepoint_t> tuple_t;

    typedef std::vector<typename D0::Ptr> ValueList;
    typedef std::vector<typename D1::Ptr> TimePointList;

    ValueList mValues;
    TimePointList mTimepoints;
    graph_analysis::BaseGraph::Ptr mpGraph;

    typedef std::pair<typename value_t::Ptr, typename timepoint_t::Ptr> ValueTimePair;
    typedef std::map< ValueTimePair, typename tuple_t::Ptr> TupleMap;
    TupleMap mTupleMap;


    TemporallyExpandedNetwork(const ValueList& values, const TimePointList& timepoints)
        : mValues(values)
        , mTimepoints(timepoints)
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
        initialize();
    }

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
                    WeightedEdge::Ptr edge(new WeightedEdge(previousTuple, currentTuple, std::numeric_limits<WeightedEdge::value_t>::max()));
                    mpGraph->addEdge(edge);
                }
                previousTuple = currentTuple;
            }
        }
    }

    const graph_analysis::BaseGraph::Ptr& getGraph() const { return mpGraph; }

    void addTuple(const typename D0::Ptr& value, const typename D1::Ptr& timepoint, const typename tuple_t::Ptr& tuple)
    {
        mTupleMap[ ValueTimePair(value, timepoint) ] = tuple;
    }

    /**
     * Retrieve a tuple (actually a graph vertex) by the given key tuple
     */
    typename tuple_t::Ptr tupleByKeys(const typename D0::Ptr& value, const typename D1::Ptr& timepoint)
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
