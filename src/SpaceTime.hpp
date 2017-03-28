#ifndef TEMPL_SPACE_TIME_NETWORK_HPP
#define TEMPL_SPACE_TIME_NETWORK_HPP

#include <templ/TemporallyExpandedNetwork.hpp>
#include <templ/symbols/constants/Location.hpp>
#include <templ/RoleInfoTuple.hpp>

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
    /// The standard representation of a space time network -- which is a
    /// temporally expanded network with locations-timepoints tuples
    /// RoleInfoTuple is associated in order to associate content with each edge
    typedef TemporallyExpandedNetwork< templ::symbols::constants::Location::Ptr,
            templ::solvers::temporal::point_algebra::TimePoint::Ptr,
            RoleInfoTuple<templ::symbols::constants::Location::Ptr,
                    templ::solvers::temporal::point_algebra::TimePoint::Ptr>
            > Network;

    typedef std::pair<symbols::constants::Location::Ptr, solvers::temporal::point_algebra::TimePoint::Ptr> Point;
    /// A timeline consisting of Points in SpaceTime
    typedef std::vector< Point > Timeline;

    /// A role-mapped number of timelines
    typedef std::map<Role, Timeline > Timelines;

    /**
     * Convert a timeline to string
     */
    static std::string toString(const Timeline& timeline, size_t indent = 0);

    /**
     * Converte a list of timelines to string
     */
    static std::string toString(const Timelines& timelines, size_t indent = 0);

};

} // end namespace templ
#endif // TEMPL_SPACE_TIME_NETWORK_HPP
