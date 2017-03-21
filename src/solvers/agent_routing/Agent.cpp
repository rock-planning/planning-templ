#include "Agent.hpp"
#include <algorithm>
#include <sstream>

namespace templ {
namespace agent_routing {

Agent::Agent()
    : mAgentId(0)
    , mTypeId(0)
{}

Agent::Agent(AgentId id, AgentTypeId typeId)
    : mAgentId(id)
    , mTypeId(typeId)
{}


void Agent::addTask(const AgentTask& task)
{
    mTasks.push_back(task);

    addLocation(task.getLocation());

    addTimePoint(task.getArrival());
    addTimePoint(task.getDeparture());
}

void Agent::addLocation(const symbols::constants::Location& location)
{
    symbols::constants::Location::List::const_iterator lit;
    lit = std::find(mLocations.begin(), mLocations.end(), location);
    if(lit == mLocations.end())
    {
        mLocations.push_back(location);
    }
    // other location has already been added
}

void Agent::addTimePoint(const solvers::temporal::point_algebra::TimePoint::Ptr& timepoint)
{
    solvers::temporal::point_algebra::TimePoint::PtrList::const_iterator tit;
    tit = std::find(mTimePoints.begin(), mTimePoints.end(), timepoint);
    if(tit == mTimePoints.end())
    {
        mTimePoints.push_back(timepoint);
    }
}

std::string Agent::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent, ' ');
    ss << hspace << "Agent:" << std::endl;
    ss << hspace << "    id: " << mAgentId << std::endl;
    ss << hspace << "    type: " << mTypeId << std::endl;
    ss << AgentTask::toString(mTasks, indent + 4);
    return ss.str();
}

std::string Agent::toString(const Agent::List& list, uint32_t indent)
{
    std::stringstream ss;
    std::string hspace(indent, ' ');
    Agent::List::const_iterator cit = list.begin();
    for(; cit != list.end(); ++cit)
    {
        ss << cit->toString(indent);
    }
    return ss.str();
}

} // end agent_routing
} // end templ
