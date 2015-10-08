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
    std::vector<solvers::temporal::Interval> mIntervals;
    organization_model::OrganizationModel::Ptr mOrganizationModel;

    Role mRole;

public:
    void setRole(const Role& role) { mRole = role; }

    void add(const solvers::csp::FluentTimeResource& fts) { mFluents.push_back(fts); }

    const std::vector<FluentTimeResource>& getFluentTimeResources() const { return mFluents; }
    const std::vector<symbols::constants::Location::Ptr>& getLocations() const { return mLocations; }
    const std::vector<solvers::temporal::Interval>& getIntervals() const { return mIntervals; }

    symbols::constants::Location::Ptr getLocation(const solvers::csp::FluentTimeResource& fts) const;
    solvers::temporal::Interval getInterval(const solvers::csp::FluentTimeResource& fts) const;

    /**
     * Sort this timeline using the set of intervals 
     * to map the time indices in the FluentTimeResource to actual intervals
     *
     * TODO: a timeline has not necessarily a unique ordering
     */
    void sortByTime();

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
