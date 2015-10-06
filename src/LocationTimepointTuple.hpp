#ifndef TEMPL_LOCATION_TIMEPOINT_TUPLE_HPP
#define TEMPL_LOCATION_TIMEPOINT_TUPLE_HPP

#include <templ/Tuple.hpp>
#include <templ/symbols/constants/Location.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>

namespace templ {

typedef Tuple< templ::symbols::constants::Location,
    templ::solvers::temporal::point_algebra::TimePoint> LocationTimepointTuple;

} // end namespace templ
#endif // TEMPL_LOCATION_TIMEPOINT_TUPLE_HPP
