#ifndef TEMPL_SPACE_TIME_NETWORK_HPP
#define TEMPL_SPACE_TIME_NETWORK_HPP

#include <limits>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <graph_analysis/BaseGraph.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/io/GraphvizWriter.hpp>
#include <graph_analysis/io/GraphvizGridStyle.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <base-logging/Logging.hpp>

#include "symbols/constants/Location.hpp"
#include "solvers/temporal/point_algebra/TimePoint.hpp"
#include "solvers/temporal/QualitativeTemporalConstraintNetwork.hpp"
#include "solvers/csp/TemporalConstraintNetwork.hpp"
#include "Tuple.hpp"

#include "RoleInfoTuple.hpp"
#include "RoleInfoWeightedEdge.hpp"

namespace templ {

class SpaceTimeNetwork
{

public:
    typedef symbols::constants::Location::Ptr value_t;
    typedef solvers::temporal::point_algebra::TimePoint::Ptr timepoint_t;

    typedef RoleInfoTuple<value_t, timepoint_t> tuple_t;
    typedef RoleInfoWeightedEdge edge_t;

    typedef std::vector<value_t> ValueList;
    typedef std::vector<timepoint_t> TimePointList;

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
    SpaceTimeNetwork();
    SpaceTimeNetwork(const SpaceTimeNetwork& other);

    /**
     * Constructor for a temporally expanded network from a set of values and
     * timepoints
     * \param values List of symbols that should be temporally qualified
     * \param timepoints List of timepoints which are assumed to be sorted
     * according to time
     * \param locationTransitionEdge type of edge which is created for a
     * location transition edge
     */
    SpaceTimeNetwork(const ValueList& values,
            const TimePointList& timepoints,
            const graph_analysis::Edge::Ptr& locationTransitionEdge = graph_analysis::Edge::Ptr());

    virtual ~SpaceTimeNetwork();

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
    void initialize();

    /**
     * Avoid the need for serialization of the tuple map, and reconstruct it
     * instead from the set of timepoints and fluent
     */
    void reconstructTupleMap();

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


    void addTuple(const value_t& value,
            const timepoint_t& timepoint,
            const typename tuple_t::Ptr& tuple);

    /**
     * Retrieve a tuple (actually a graph vertex) by the given key tuple
     * \param value
     * \param timepoint
     * \return tuple
     */
    typename tuple_t::Ptr tupleByKeys(const value_t& value, const timepoint_t& timepoint) const;

    void save(const std::string& filename, const std::string& type = "") const;

    static SpaceTimeNetwork fromFile(const std::string& filename,
            const std::vector<value_t>& values,
            const std::vector<timepoint_t>& timepoints);

    static SpaceTimeNetwork fromFile(const std::string& filename);

    static SpaceTimeNetwork fromGraph(const graph_analysis::BaseGraph::Ptr& graph, const std::vector<value_t>& values, const std::vector<timepoint_t>& timepoints);

    const ValueTimePair& getValueTimePair(const typename tuple_t::Ptr& searchTuple) const;

    const SpaceTimeNetwork::value_t& getValue(const typename tuple_t::Ptr& tuple) const;
    const SpaceTimeNetwork::timepoint_t& getTimepoint(const typename tuple_t::Ptr& tuple) const;

    size_t getColumn(const graph_analysis::Vertex::Ptr& vertex) const;

    size_t getRow(const graph_analysis::Vertex::Ptr& vertex) const;

    /**
     * Get all timepoints that are part of a given interval
     */
    TimePointList getTimepoints(const timepoint_t& t_start, const timepoint_t& t_end) const;

    /**
     * Get all tuples for a given interval and a given temporally
     * expanded value
     * \param t_start start timepoint of the interval
     * \param t_end end timepoint of the interval
     * \param value value (e.g. location symbol) which has a temporal expansion
     * \return list of tuples pointers
     */
    TuplePtrList getTuples(const timepoint_t& t_start,
            const timepoint_t& t_end,
            const value_t& value) const;
};

} // end namespace templ
#endif // TEMPL_SPACE_TIME_NETWORK_HPP
