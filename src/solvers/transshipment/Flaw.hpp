#ifndef TEMPL_SOLVERS_TRANSHIPMENT_FLAW_HPP
#define TEMPL_SOLVERS_TRANSHIPMENT_FLAW_HPP

#include <templ/solvers/csp/RoleTimeline.hpp>
#include <graph_analysis/algorithms/MultiCommodityMinCostFlow.hpp>

namespace templ {
namespace solvers {
namespace transshipment {

/**
 * A Flaw represents a violation of the current solution with respect to the
 * requirements
 */
struct Flaw
{
    graph_analysis::algorithms::ConstraintViolation violation;
    Role::List affectedRoles;
    csp::RoleTimeline roleTimeline;

    FluentTimeResource previousFtr;
    FluentTimeResource ftr;
    FluentTimeResource subsequentFtr;

    std::string description;

    const Role& affectedRole() const;

    Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
        const Role& role);

    Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
        const Role::List& roles);

    std::string toString(size_t indent = 0) const;
};

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TRANSHIPMENT_FLAW_HPP
