#include "AgentTask.hpp"
#include <limits>
#include <sstream>

namespace templ {
namespace solvers {
namespace agent_routing {

AgentTask::AgentTask()
    : mPriority(0)
    , mLocation()
    , mTaskDuration(0)
{
}

std::string AgentTask::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent, ' ');
    ss << hspace << "AgentTask:" << std::endl;
    ss << hspace << "    priority:" << mPriority << std::endl;
    ss << hspace << "    duration:" << mTaskDuration << std::endl;
    ss << hspace << "    location: " << mLocation.toString() << std::endl;
    if(mArrival)
    {
        ss << hspace << "    arrival: " << mArrival->toString() << std::endl;
    } else {
        ss << hspace << "    arrival: n/a" << std::endl;
    }
    if(mDeparture)
    {
        ss << hspace << "    departure: " << mDeparture->toString() << std::endl;
    } else {
        ss << hspace << "    departure: n/a" << std::endl;
    }
    return ss.str();
}

std::string AgentTask::toString(const AgentTask::List& list, uint32_t indent)
{
    std::stringstream ss;

    AgentTask::List::const_iterator cit = list.begin();
    for(; cit != list.end(); ++cit)
    {
        ss << cit->toString(indent);
    }
    return ss.str();
}

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
