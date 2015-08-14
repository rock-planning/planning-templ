#include "Chronicle.hpp"
#include <base/Logging.hpp>

namespace templ {
namespace solvers {
namespace temporal {

void Chronicle::addTimeline(const Timeline& timeline)
{
    symbols::StateVariable stateVar = timeline.getStateVariable();
    std::map<symbols::StateVariable, Timeline>::iterator it = mTimelines.find(stateVar);
    if(it != mTimelines.end())
    {
        throw std::invalid_argument("templ::solvers::temporal::Chronicle::addTimeline: timeline has already been added");
    } 
    mTimelines[stateVar] = timeline;
}

bool Chronicle::isConsistent() const
{
    std::map<symbols::StateVariable, Timeline>::const_iterator cit = mTimelines.begin();
    for(; cit != mTimelines.end(); ++cit)
    {
        const Timeline& timeline = cit->second;
        if(!timeline.isConsistent())
        {
            LOG_DEBUG_S << "Timeline is inconsistent: " << timeline.toString();
            return false;
        }
    }
    return true;
}

std::string Chronicle::toString() const
{
    std::stringstream ss;
    ss << "BEGIN Chronicle ---" << std::endl;
    std::map<symbols::StateVariable, Timeline>::const_iterator cit = mTimelines.begin();
    for(; cit != mTimelines.end(); ++cit)
    {
        const Timeline& timeline = cit->second;
        ss << timeline.toString();
    }
    ss << "END Chronicle ---" << std::endl;
    return ss.str();
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
