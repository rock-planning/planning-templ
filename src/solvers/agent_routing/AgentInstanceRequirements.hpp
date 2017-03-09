#ifndef TEMPL_AGENT_ROUTING_AGENT_INSTANCE_REQUIREMENTS_HPP
#define TEMPL_AGENT_ROUTING_AGENT_INSTANCE_REQUIREMENTS_HPP

#include <cstdint>
#include "AgentTask.hpp"

namespace templ {
namespace agent_routing {

typedef uint32_t AgentId;
typedef uint32_t AgentTypeId;

class AgentInstanceRequirement
{
public:
    //AgentInstanceRequirement();

    void setAgentType(const AgentTypeId& agentType) { mTypeId = agentType; }
    AgentTypeId getAgentType() const { return mTypeId; }

    void setAgentId(const AgentId& agentId) { mAgentId = agentId; }
    AgentId getAgentId() const { return mAgentId; }

    void addTask(const AgentTask& task);
    const std::vector<AgentTask>& getAgentTasks() const { return mTasks; }



private:
    AgentId mAgentId;
    AgentTypeId mTypeId;

    std::vector<AgentTask> mTasks;
};

} // end namespace agent_routing
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_INSTANCE_REQUIREMENTS_HPP
