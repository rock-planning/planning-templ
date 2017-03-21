#ifndef TEMPL_AGENT_ROUTING_AGENT_TASK_HPP
#define TEMPL_AGENT_ROUTING_AGENT_TASK_HPP

#include <cstdint>
#include <templ/symbols/constants/Location.hpp>
#include <templ/solvers/temporal/Interval.hpp>

namespace templ {
namespace solvers {
namespace agent_routing {

typedef uint32_t TaskPriority;
typedef uint32_t TaskDuration;

class AgentTask
{
public:
    typedef std::vector<AgentTask> List;

    AgentTask();

    void setTaskPriority(TaskPriority p) { mPriority = p; }
    TaskPriority getTaskPriority() const { return mPriority; }

    void setLocation(const symbols::constants::Location& c) { mLocation = c; }
    const symbols::constants::Location& getLocation() const { return mLocation; }

    void setTaskDuration(const TaskDuration& t) { mTaskDuration = t; }
    TaskDuration getTaskDuration() const { return mTaskDuration; }

    void setArrival(const solvers::temporal::point_algebra::TimePoint::Ptr& t) { mArrival = t; }
    solvers::temporal::point_algebra::TimePoint::Ptr getArrival() const { return mArrival; }

    void setDeparture(const solvers::temporal::point_algebra::TimePoint::Ptr& t) { mDeparture = t; }
    solvers::temporal::point_algebra::TimePoint::Ptr getDeparture() const { return mDeparture; }

    std::string toString(uint32_t indent = 0) const;
    static std::string toString(const AgentTask::List& list, uint32_t uindent);

private:
    TaskPriority mPriority;
    symbols::constants::Location mLocation;
    TaskDuration mTaskDuration;
    solvers::temporal::point_algebra::TimePoint::Ptr mArrival;
    solvers::temporal::point_algebra::TimePoint::Ptr mDeparture;
};

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_TASK_HPP
