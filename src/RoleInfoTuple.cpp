#include "RoleInfoTuple.hpp"
#include "symbols/constants/Location.hpp"
#include "solvers/temporal/point_algebra/TimePoint.hpp"

namespace templ {

template<>
const graph_analysis::VertexRegistration< RoleInfoTuple<templ::symbols::constants::Location::Ptr,
                    templ::solvers::temporal::point_algebra::TimePoint::Ptr> >
            RoleInfoTuple<templ::symbols::constants::Location::Ptr,
                    templ::solvers::temporal::point_algebra::TimePoint::Ptr>::msRoleInfoTupleRegistration = graph_analysis::VertexRegistration< RoleInfoTuple<templ::symbols::constants::Location::Ptr, templ::solvers::temporal::point_algebra::TimePoint::Ptr> >();
}
