#include "RoleTimeline.hpp"
#include <organization_model/facades/Robot.hpp>
#include <base-logging/Logging.hpp>
#include "../Cost.hpp"

namespace templ {
namespace solvers {
namespace csp {

symbols::constants::Location::Ptr RoleTimeline::getLocation(const solvers::FluentTimeResource& fts) const
{
    return mLocations.at(fts.getFluentIdx());
}

solvers::temporal::Interval RoleTimeline::getInterval(const solvers::FluentTimeResource& fts) const
{
    return mIntervals.at(fts.getTimeIntervalIdx());
}

bool RoleTimeline::operator<(const RoleTimeline& other) const
{
    if(mFluents < other.mFluents)
    {
        return true;
    } else if(mFluents == other.mFluents)
    {
        if(mLocations < other.mLocations)
        {
            return true;
        } else if(mLocations == other.mLocations)
        {
            if(mIntervals < other.mIntervals)
            {
                return true;
            } else if(mIntervals == other.mIntervals)
            {
                if(mRole < other.mRole)
                {
                    return true;
                } else if(mRole == other.mRole)
                {
                    return false;
                }
            }
        }
    }
    return false;
}

void RoleTimeline::sortByTime()
{
    using namespace solvers::csp;
    if(mIntervals.empty())
    {
        throw std::runtime_error("templ::solvers::csp::RoleTimeline::sortByTime: intervals not set");
    }

    std::sort( mFluents.begin(), mFluents.end(), [this](const FluentTimeResource& a, const FluentTimeResource& b)->bool
            {
                if(a == b)
                {
                    return false;
                }

                const solvers::temporal::Interval& lval = this->mIntervals.at(a.getTimeIntervalIdx());
                const solvers::temporal::Interval& rval = this->mIntervals.at(b.getTimeIntervalIdx());
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

    organization_model::OrganizationModelAsk ask(mOrganizationModel);
    organization_model::facades::Robot robot(mRole.getModel(), ask);
    ss << "    " << robot.toString() << std::endl;

    ss << "    travel distance (in km):         " << travelDistance()/1.0E3 << std::endl;
    ss << "    estimated time (in h):         " << duration()/3.6E3 << std::endl;
    ss << "    estimated energy cost (in kWh):  " << estimatedEnergyCost()/1.0E3 << std::endl;
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
        timeline.mIntervals = intervals;

        timeline.sortByTime();
    }

    return timelines;
}

std::string RoleTimeline::toString(const std::map<Role, RoleTimeline>& timelines, uint32_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    std::map<Role, RoleTimeline>::const_iterator it = timelines.begin();
    for(; it != timelines.end(); ++it)
    {
        const RoleTimeline& timeline = it->second;
        ss << hspace << timeline.toString();
    }
    return ss.str();
}

double RoleTimeline::travelDistance() const
{
    using namespace ::templ::symbols;
    if(mFluents.empty())
    {
        throw std::runtime_error("templ::solvers::csp::RoleTimeline::travelDistance:"
                " no fluents available to compute travel distance");
    }

    constants::Location::PtrList path;
    std::vector<FluentTimeResource>::const_iterator cit = mFluents.begin();
    for(; cit != mFluents.end(); ++cit)
    {
        const FluentTimeResource& ftr = *cit;
        path.push_back(ftr.getLocation());
    }
    return Cost::getTravelDistance(path);
}

double RoleTimeline::estimatedEnergyCost() const
{
    double distance = travelDistance();

    organization_model::OrganizationModelAsk ask(mOrganizationModel);
    organization_model::facades::Robot robot(mRole.getModel(), ask);
    return robot.estimatedEnergyCost(distance);
}

double RoleTimeline::duration() const
{
    organization_model::OrganizationModelAsk ask(mOrganizationModel);
    organization_model::facades::Robot robot(mRole.getModel(), ask);
    return travelDistance()/robot.getNominalVelocity();
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
