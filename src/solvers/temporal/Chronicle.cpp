#include "Chronicle.hpp"

namespace templ {
namespace solvers {
namespace temporal {

void Chronicle::addTimeline(const Timeline& timeline)
{
    StateVariable stateVar = timeline.getStateVariable();
    std::map<StateVariable, Timeline>::iterator it = mTimelines.find(stateVar);
    if(it != mTimelines.end())
    {
        throw std::invalid_argument("templ::solvers::temporal::Chronicle::addTimeline: timeline has already been added");
    } 
    mTimelines[stateVar] = timeline;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
