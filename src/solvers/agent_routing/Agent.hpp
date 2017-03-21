#ifndef TEMPL_AGENT_ROUTING_AGENT_HPP
#define TEMPL_AGENT_ROUTING_AGENT_HPP

#include <cstdint>
#include "AgentTask.hpp"
#include "AgentType.hpp"

namespace templ {
namespace agent_routing {

typedef uint32_t AgentId;

class Agent
{
public:
    typedef std::vector<Agent> List;

    Agent();

    Agent(AgentId id, AgentTypeId typeId);

    void setAgentType(const AgentTypeId& agentType) { mTypeId = agentType; }
    AgentTypeId getAgentType() const { return mTypeId; }

    void setAgentId(const AgentId& agentId) { mAgentId = agentId; }
    AgentId getAgentId() const { return mAgentId; }

    void addTask(const AgentTask& task);
    const AgentTask::List& getAgentTasks() const { return mTasks; }

    /**
     * Get all locations this agent is assigned through its tasks
     */
    const symbols::constants::Location::List& getLocations() const { return mLocations; }

    /**
     * Get all timepoints this agent is sensitive towards
     */
     const solvers::temporal::point_algebra::TimePoint::PtrList& getTimePoints() const { return mTimePoints; }

     std::string toString(uint32_t indent = 0) const;

     static std::string toString(const List& list, uint32_t indent = 0);

private:
    AgentId mAgentId;
    AgentTypeId mTypeId;

    AgentTask::List mTasks;
    symbols::constants::Location::List mLocations;
    solvers::temporal::point_algebra::TimePoint::PtrList mTimePoints;

    void addLocation(const symbols::constants::Location& location);

    void addTimePoint(const solvers::temporal::point_algebra::TimePoint::Ptr& timepoint);

};

} // end namespace agent_routing
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_HPP
