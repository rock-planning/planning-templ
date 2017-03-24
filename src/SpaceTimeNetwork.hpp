#ifndef TEMPL_SPACE_TIME_NETWORK_HPP
#define TEMPL_SPACE_TIME_NETWORK_HPP

#include <templ/TemporallyExpandedNetwork.hpp>
#include <templ/symbols/constants/Location.hpp>
#include <templ/RoleInfoTuple.hpp>

namespace templ
{
    /// The standard representation of a space time network -- which is a
    /// temporally expanded network with locations-timepoints tuples
    /// RoleInfoTuple is associated in order to associate content with each edge
    typedef TemporallyExpandedNetwork< templ::symbols::constants::Location::Ptr,
            templ::solvers::temporal::point_algebra::TimePoint::Ptr,
            RoleInfoTuple<templ::symbols::constants::Location::Ptr,
                    templ::solvers::temporal::point_algebra::TimePoint::Ptr>
            > SpaceTimeNetwork;
} // end namespace templ
#endif // TEMPL_SPACE_TIME_NETWORK_HPP
