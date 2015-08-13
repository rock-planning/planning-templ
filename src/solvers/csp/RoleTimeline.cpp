#include "RoleTimeline.hpp"

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
            timelines[role].setRole(role);
            timelines[role].add(fts);
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

} // end namespace csp
} // end namespace solvers
} // end namespace templ
