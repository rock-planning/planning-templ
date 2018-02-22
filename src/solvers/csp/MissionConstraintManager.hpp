#ifndef TEMPL_SOLVERS_CSP_MISSION_CONSTRAINT_MANAGER_HPP
#define TEMPL_SOLVERS_CSP_MISSION_CONSTRAINT_MANAGER_HPP

#include "../../SharedPtr.hpp"
#include "../../Constraint.hpp"
#include "../FluentTimeResource.hpp"
#include "../../SpaceTime.hpp"

namespace templ {
namespace constraints {
    class ModelConstraint;
}
}

namespace templ {
namespace solvers {
namespace csp {

class TransportNetwork;

class MissionConstraintManager
{
public:
    static void apply(const Constraint::Ptr& constraint, TransportNetwork& transportNetwork);

    static void apply(const shared_ptr<constraints::ModelConstraint>& constraint, TransportNetwork& transportNetwork);

    static std::vector<SpaceTime::SpaceIntervalTuple> mapToSpaceTime(const FluentTimeResource::List& ftrs);

private:
    static FluentTimeResource::List findAffected(const shared_ptr<constraints::ModelConstraint>& constraint, const FluentTimeResource::List& ftrs);

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_MISSION_CONSTRAINT_MANAGER_HPP
