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
    AgentInstanceRequirement();

    AgentTypeId getAgentType() const { return mTypeId; }
    AgentId getAgentId() const { return mAgentId; }

    void addTask(const AgentTask& task);



private:
    AgentId mAgentId;
    AgentTypeId mTypeId;

    std::vector<AgentTask> mTasks;
};

} // end namespace agent_routing
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_INSTANCE_REQUIREMENTS_HPP
