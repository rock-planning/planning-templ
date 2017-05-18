#include "SpaceTime.hpp"
#include <sstream>

namespace templ
{

std::string SpaceTime::toString(const Timeline& timeline, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << "TIMELINE --" << std::endl;
    Timeline::const_iterator cit = timeline.begin();
    for(; cit != timeline.end(); ++cit)
    {
        const Point& stp = *cit;
        assert(stp.first);
        if(stp.first)
        {
            ss << hspace <<  stp.first->toString();
        } else {
            ss << " -- " << "unknown location";
        }

        if(stp.second)
        {
            ss << " -- " << stp.second->toString();
        } else {
            ss << " -- " << "unknown timepoint";
        }

        ss << std::endl;
    }
    return ss.str();
}

std::string SpaceTime::toString(const Timelines& timelines, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << "TIMELINES --" << std::endl;

    Timelines::const_iterator cit = timelines.begin();
    for(; cit != timelines.end(); ++cit)
    {
        const Role& role = cit->first;
        const Timeline& timeline = cit->second;

        ss << hspace << role.toString() << std::endl;
        ss << toString(timeline, indent + 4);
    }
    return ss.str();
}

SpaceTime::Network SpaceTime::toNetwork(const symbols::constants::Location::PtrList& locations,
        const templ::solvers::temporal::point_algebra::TimePoint::PtrList timepoints,
        const Timelines& timelines)
{
    Network network(locations, timepoints);
    Timelines::const_iterator cit = timelines.begin();
    for(; cit != timelines.end(); ++cit)
    {
        const Role& role = cit-> first;
        const Timeline& timeline = cit->second;

        Timeline::const_iterator tit = timeline.begin();
        for(; tit != timeline.end(); ++tit)
        {
            const Point& stp = *tit;
            const symbols::constants::Location::Ptr& location = stp.first;
            const solvers::temporal::point_algebra::TimePoint::Ptr& timepoint = stp.second;

            Network::tuple_t::Ptr roleInfo = network.tupleByKeys(location, timepoint);
            roleInfo->addRole(role);
        }
    }
    return network;
}

} // end namespace templ
