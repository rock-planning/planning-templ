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

} // end namespace templ
