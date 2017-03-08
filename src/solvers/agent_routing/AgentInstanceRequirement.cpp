#include "AgentInstanceRequirements.hpp"

namespace templ {
namespace agent_routing {

void AgentInstanceRequirements::addTask(const AgentTask& task)
{
    mTasks.push_back(task);
}

} // end agent_routing
} // end templ
