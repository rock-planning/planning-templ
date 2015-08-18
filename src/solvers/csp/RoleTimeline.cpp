#include "RoleTimeline.hpp"
#include <organization_model/facets/Robot.hpp>

namespace templ {
namespace solvers {
namespace csp {

void RoleTimeline::sort(const std::vector<solvers::temporal::Interval>& intervals)
{
    using namespace solvers::csp;
    std::sort( mFluents.begin(), mFluents.end(), [&intervals](const FluentTimeResource& a, const FluentTimeResource& b)->bool
            {
                if(a == b)
                {
                    return false;
                }

                const solvers::temporal::Interval& lval = intervals.at(a.time);
                const solvers::temporal::Interval& rval = intervals.at(b.time);
                return lval.before(rval);
            });
}

std::string RoleTimeline::toString() const
{
    std::stringstream ss;
    ss << "-- Timeline: " << mRole.toString() << std::endl;
    for(auto &fluent : mFluents)
    {
        ss << fluent.toString() << std::endl;
    }
    ss << "    travel distance (in km):       " << travelDistance()/1.0E3;
    ss << "    estimated energy cost (in Wh): " << estimatedEnergyCost();
    return ss.str();
}


std::map<Role,RoleTimeline> RoleTimeline::computeTimelines(const Mission& mission, const RoleDistribution::Solution& roleSolution)
{
    std::map<Role,RoleTimeline> timelines;

    // Timeline
    RoleDistribution::Solution::const_iterator cit = roleSolution.begin();
    for(; cit != roleSolution.end(); ++cit)
    {
        const Role::List& roles = cit->second;
        const FluentTimeResource& fts = cit->first;

        Role::List::const_iterator lit = roles.begin();
        for(; lit != roles.end(); ++lit)
        {
            const Role& role = *lit;
            RoleTimeline& timeline = timelines[role];
            timeline.setRole(role);
            timeline.add(fts);
            timeline.mLocations = mission.getLocations();
            timeline.mOrganizationModel = mission.getOrganizationModel();
        }
    }

    // Sort timeline
    std::vector<solvers::temporal::Interval> intervals(mission.getTimeIntervals().begin(), mission.getTimeIntervals().end());

    std::map<Role, RoleTimeline>::iterator it = timelines.begin();
    for(; it != timelines.end(); ++it)
    {
        RoleTimeline& timeline = it->second;
        timeline.sort(intervals);
    }

    return timelines;
}

double RoleTimeline::travelDistance() const
{
    using namespace ::templ::symbols;
    constants::Location::Ptr fromLocation;
    constants::Location::Ptr toLocation;

    std::vector<FluentTimeResource>::const_iterator cit = mFluents.begin();
    double totalDistance;
    for(; cit != mFluents.end(); ++cit)
    {
        const FluentTimeResource& ftr = *cit;
        if(!fromLocation)
        {
            fromLocation = mLocations[ftr.fluent];
            continue;
        }
        toLocation = mLocations[ftr.fluent];
        totalDistance += (toLocation->getPosition() - fromLocation->getPosition()).norm();

        fromLocation = toLocation;
    }
    return totalDistance;
}

double RoleTimeline::estimatedEnergyCost() const
{
    organization_model::facets::Robot robot(mRole.getModel(), mOrganizationModel);
    double distance = travelDistance();
    return robot.estimatedEnergyCost(distance);
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
