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
    Role affectedRole;
    csp::RoleTimeline roleTimeline;

    csp::FluentTimeResource previousFtr;
    csp::FluentTimeResource ftr;
    csp::FluentTimeResource subsequentFtr;

    std::string description;

    Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
        const Role& role)
        : violation(violation)
        , affectedRole(role)
    {}

    std::string toString(size_t indent = 0) const;
};

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TRANSHIPMENT_FLAW_HPP
