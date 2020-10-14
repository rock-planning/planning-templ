#ifndef TEMPL_SPACE_TIME_HPP
#define TEMPL_SPACE_TIME_HPP

#include "SpaceTimeNetwork.hpp"
#include "symbols/constants/Location.hpp"
#include "RoleInfoTuple.hpp"
#include "RoleInfoWeightedEdge.hpp"
#include "solvers/temporal/Interval.hpp"

namespace templ
{

/**
 * \class SpaceTime represents a utility class allowing to deal with the
 * locations that are associated with timepoints, allowing a discretized representation
 * of space-time
 */
class SpaceTime
{
public:

    typedef Tuple<templ::symbols::constants::Location::Ptr,
        templ::solvers::temporal::point_algebra::TimePoint::Ptr> SpaceTimeTuple;

    typedef Tuple<templ::symbols::constants::Location::Ptr,
        templ::solvers::temporal::Interval> SpaceIntervalTuple;

    typedef RoleInfoTuple<templ::symbols::constants::Location::Ptr,
        templ::solvers::temporal::point_algebra::TimePoint::Ptr> RoleInfoSpaceTimeTuple;

    /// The standard representation of a space time network -- which is a
    /// temporally expanded network with locations-timepoints tuples
    /// RoleInfoTuple is associated in order to associate content with each edge
    //typedef TemporallyExpandedNetwork< templ::symbols::constants::Location::Ptr,
    //        templ::solvers::temporal::point_algebra::TimePoint::Ptr,
    //        RoleInfoSpaceTimeTuple,
    //        RoleInfoWeightedEdge
    //        > Network;
    typedef templ::SpaceTimeNetwork Network;

    /// A route through the space time network
    typedef std::vector< shared_ptr<RoleInfoSpaceTimeTuple> > Route;

    typedef std::pair<symbols::constants::Location::Ptr, solvers::temporal::point_algebra::TimePoint::Ptr> Point;

    typedef std::vector<Point> Points;

    /// A timeline equals Points in SpaceTime
    typedef Points Timeline;

    /// A role-mapped number of timelines
    typedef std::map<Role, Timeline > Timelines;

    /**
     * Get the time horizon's starting point
     * \return timepoint describing the start time
     */
    static SpaceTime::Network::timepoint_t getHorizonStart();

    /**
     * Get the time horizon's end point
     * \return timepoint describing the end time
     */
    static SpaceTime::Network::timepoint_t getHorizonEnd();

    /**
     * Get the value describing the depot
     * \return location representing the depot
     */
    static SpaceTime::Network::value_t getDepot();

    /**
     * Return space time tuple representing the intial depot
     * \return tuple for start depot
     */
    static SpaceTime::Network::tuple_t::Ptr getHorizonStartTuple();

    /**
     * Return space time tuple representing the final depot
     * \return tuple for final depot
     */
    static SpaceTime::Network::tuple_t::Ptr getHorizonEndTuple();

    /**
     * Convert spacetime to string
     */
    static std::string toString(const Point& spacetime, size_t indent = 0);

    /**
     * Convert a timeline to string
     */
    static std::string toString(const Timeline& timeline, size_t indent = 0);

    /**
     * Convert a list of timelines to string
     */
    static std::string toString(const Timelines& timelines, size_t indent = 0);

    /**
     * Convert to a set of timelines to a graph representation, timelines
     * \return SpaceTime::Network
     */
    static SpaceTime::Network toNetwork(const symbols::constants::Location::PtrList& locations, const templ::solvers::temporal::point_algebra::TimePoint::PtrList& timepoints, const Timelines& timelines);

    /**
     * Append a spacetimepoint to a timeline (only when it is not already part of the
     * timeline)
     */
    static void appendToTimeline(Timeline& timeline, const Point& point);

    /**
     * Insert virtual start and end depot markers, e.g., to allow flow
     * optimization to deal with single start and end depot
     * \see getHorizonStartTuple
     * \see getHorizonEndTuple
     */
    static void injectVirtualStartAndEnd(SpaceTime::Network& network);

    /**
     * Test whether interval starts at start horizon and ends at end horizon of,
     * i.e. covers the full mission
     * \return true if the full mission is covered, false otherwise
     */
    static bool isFullMissionInterval(const solvers::temporal::Interval& interval);

    /**
     * Sort a timeline according to time
     */
    static void sort(Timeline& timeline,
            const solvers::temporal::point_algebra::TimePointComparator& tpc);

private:
    static SpaceTime::Network::tuple_t::Ptr msHorizonStartTuple;
    static SpaceTime::Network::tuple_t::Ptr msHorizonEndTuple;

};

} // end namespace templ
#endif // TEMPL_SPACE_TIME_HPP
