#ifndef TEMPL_AGENT_ROUTING_AGENT_TASK_HPP
#define TEMPL_AGENT_ROUTING_AGENT_TASK_HPP

#include <cstdint>
#include <templ/symbols/constants/Location.hpp>
#include <templ/solvers/temporal/Interval.hpp>

namespace templ {
namespace agent_routing {

typedef uint32_t TaskPriority;
typedef uint32_t TaskDuration;
typedef solvers::temporal::Interval TimeWindow;

class AgentTask
{
public:

    AgentTask();

    void setTaskPriority(TaskPriority p) { mPriority = p; }
    TaskPriority getTaskPriority() const { return mPriority; }

    void setLocation(const symbols::constants::Location& c) { mLocation = c; }
    const symbols::constants::Location& getLocation() const { return mLocation; }

    void setTaskDuration(const TaskDuration& t) { mTaskDuration = t; }
    TaskDuration getTaskDuration() const { return mTaskDuration; }

    void setArrival(const TimeWindow& t) { mArrival = t; }
    TimeWindow getArrival() const { return mArrival; }

    void setDeparture(const TimeWindow& t) { mDeparture = t; }
    TimeWindow getDeparture() const { return mDeparture; }

private:
    TaskPriority mPriority;
    symbols::constants::Location mLocation;
    TaskDuration mTaskDuration;
    TimeWindow mArrival;
    TimeWindow mDeparture;
};

} // end namespace agent_routing
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_TASK_HPP
