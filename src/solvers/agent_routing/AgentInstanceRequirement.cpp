#include "AgentInstanceRequirements.hpp"

namespace templ {
namespace agent_routing {

void AgentInstanceRequirement::addTask(const AgentTask& task)
{
    mTasks.push_back(task);
}

} // end agent_routing
} // end templ
