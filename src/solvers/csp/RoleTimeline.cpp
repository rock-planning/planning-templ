#include "RoleTimeline.hpp"
#include <moreorg/facades/Robot.hpp>
#include <base-logging/Logging.hpp>
#include "../Cost.hpp"

namespace templ {
namespace solvers {
namespace csp {

RoleTimeline::RoleTimeline()
{}

RoleTimeline::RoleTimeline(const Role& role,
        const moreorg::OrganizationModelAsk& ask)
    : mRole(role)
    , mAsk(ask)
    , mRobot(moreorg::facades::Robot::getInstance(mRole.getModel(), mAsk))
{}

bool RoleTimeline::operator<(const RoleTimeline& other) const
{
    if(mRole == other.mRole)
    {
        return mTimeline < other.mTimeline;
    }
    return mRole < other.mRole;
}

void RoleTimeline::sort(const temporal::point_algebra::TimePointComparator& tpc)
{
    using namespace solvers::csp;
    SpaceTime::sort(mTimeline, tpc);
}

std::string RoleTimeline::toString(size_t indent, bool withStats) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "Timeline for '" << mRole.toString() << "'" << std::endl;
    ss << SpaceTime::toString(mTimeline,indent + 4) << std::endl;
    if(withStats)
    {
        ss << hspace << mRobot.toString() << std::endl;
        ss << hspace << "    travel distance (in km):         " << travelDistance()/1.0E3 << std::endl;
        ss << hspace << "    estimated time (in h):         " << duration()/3.6E3 << std::endl;
        ss << hspace << "    estimated energy cost (in kWh):  " << estimatedEnergyCost()/1.0E3 << std::endl;
    }
    return ss.str();
}

std::string RoleTimeline::toString(const std::map<Role, RoleTimeline>& timelines,
        uint32_t indent,
        bool noStats)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    std::map<Role, RoleTimeline>::const_iterator it = timelines.begin();
    for(; it != timelines.end(); ++it)
    {
        const RoleTimeline& timeline = it->second;
        ss << timeline.toString(indent + 4, noStats);
        ss << std::endl;
    }
    return ss.str();
}

SpaceTime::Timelines RoleTimeline::collectTimelines(const std::map<Role, RoleTimeline>& timelines)
{
    SpaceTime::Timelines spaceTimeTimelines;
    for(const std::pair<const Role, RoleTimeline>& p : timelines)
    {
        spaceTimeTimelines[p.first] = p.second.getTimeline();
    }
    return spaceTimeTimelines;
}

double RoleTimeline::travelDistance() const
{
    using namespace ::templ::symbols;
    if(mTimeline.empty())
    {
        LOG_WARN_S << "Timeline is empty -- return distance: 0 m";
        return 0.0;
    }

    constants::Location::PtrList path;
    for(const SpaceTime::Point& p : mTimeline)
    {
        path.push_back(p.first);
    }
    return Cost::getTravelDistance(path);
}

double RoleTimeline::estimatedEnergyCost() const
{
    double distance = travelDistance();

    return mRobot.estimatedEnergyCost(distance);
}

double RoleTimeline::duration() const
{
    return travelDistance()/mRobot.getNominalVelocity();
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
