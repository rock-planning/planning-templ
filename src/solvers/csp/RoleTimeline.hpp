#ifndef TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP
#define TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP

#include <map>
#include "../FluentTimeResource.hpp"
#include "../../SpaceTime.hpp"

namespace templ {
namespace solvers {
namespace csp {

class RoleTimeline
{
public:
    void setRole(const Role& role) { mRole = role; }

    void add(const solvers::FluentTimeResource& fts) { mFluents.push_back(fts); }

    const std::vector<FluentTimeResource>& getFluentTimeResources() const { return mFluents; }

    //const std::vector<symbols::constants::Location::Ptr>& getLocations() const { return mLocations; }
    //const std::vector<solvers::temporal::Interval>& getIntervals() const { return mIntervals; }


    bool operator<(const RoleTimeline& other) const;

    /**
     * Sort this timeline using the set of intervals
     * to map the time indices in the FluentTimeResource to actual intervals
     *
     * TODO: a timeline has not necessarily a unique ordering
     */
    void sortByTime();

    const SpaceTime::Timeline& getTimeline();

    std::string toString() const;

    /**
     * Compute the timelines for all roles from a given solution
     * The resulting timelines are already sorted according to time
     * \return map of roles to timelines
     */
    static std::map<Role,RoleTimeline> computeRoleTimelines(const Mission& mission, const std::map<FluentTimeResource, Role::List>& solution);

    /**
     * Convert a role->timeline map to a string representation
     * \return timeline representation
     */
    static std::string toString(const std::map<Role, RoleTimeline>& timelines, uint32_t indent = 0);

    double travelDistance() const;

    double estimatedEnergyCost() const;

    double duration() const;
private:
    std::vector<FluentTimeResource> mFluents;
    std::vector<symbols::constants::Location::Ptr> mLocations;
    std::vector<solvers::temporal::Interval> mIntervals;
    organization_model::OrganizationModel::Ptr mOrganizationModel;

    Role mRole;
    mutable SpaceTime::Timeline mTimeline;

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP
