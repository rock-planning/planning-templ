#ifndef TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP
#define TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP

#include <map>
#include <templ/solvers/csp/RoleDistribution.hpp>

namespace templ {
namespace solvers {
namespace csp {

class RoleTimeline
{
    std::vector<FluentTimeResource> mFluents;
    std::vector<symbols::constants::Location::Ptr> mLocations;
    organization_model::OrganizationModel::Ptr mOrganizationModel;

    Role mRole;

public:
    void setRole(const Role& role) { mRole = role; }

    void add(const solvers::csp::FluentTimeResource& fts) { mFluents.push_back(fts); }

    /**
     * Sort his timeline given the set of intervals 
     * which allow to map back the indices in the FluentTimeResource
     *
     */
    void sort(const std::vector<solvers::temporal::Interval>& intervals);

    std::string toString() const;

    static std::map<Role,RoleTimeline> computeTimelines(const Mission& mission, const RoleDistribution::Solution& solution);

    double travelDistance() const;

    double estimatedEnergyCost() const;

    double duration() const;
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP
