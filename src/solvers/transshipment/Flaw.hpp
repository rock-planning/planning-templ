#ifndef TEMPL_SOLVERS_TRANSHIPMENT_FLAW_HPP
#define TEMPL_SOLVERS_TRANSHIPMENT_FLAW_HPP

#include <graph_analysis/algorithms/MultiCommodityMinCostFlow.hpp>
#include "../csp/RoleTimeline.hpp"
#include "../../SpaceTime.hpp"

namespace templ {
namespace solvers {
namespace transshipment {

/**
 * A Flaw represents a violation of the current solution with respect to the
 * requirements
 */
class Flaw
{
public:
    Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
        const Role& role,
        SpaceTime::Point at);

    Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
        const Role::List& roles,
        SpaceTime::Point at);

    std::string toString(size_t indent = 0) const;

    graph_analysis::algorithms::ConstraintViolation getViolation() const { return mViolation; }
    void setViolation(const graph_analysis::algorithms::ConstraintViolation& violation) { mViolation = violation; }

    const Role& affectedRole() const;
    void setAffectedRoles(const Role::List& roles) { mAffectedRoles = roles; }

    const csp::RoleTimeline& getRoleTimeline() const { return mRoleTimeline; }
    void setRoleTimeline(const csp::RoleTimeline& timeline) { mRoleTimeline = timeline; }

    const SpaceTime::Point& getSpaceTime() const { return mSpacetime; }
    void setSpaceTime(const SpaceTime::Point& stp) { mSpacetime = stp; }

    const std::string& getDescription() const { return mDescription; }
    void setDescription(const std::string& description) { mDescription = description; }

private:
    graph_analysis::algorithms::ConstraintViolation mViolation;
    Role::List mAffectedRoles;
    csp::RoleTimeline mRoleTimeline;

    SpaceTime::Point mSpacetime;

    std::string mDescription;

};

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TRANSHIPMENT_FLAW_HPP
