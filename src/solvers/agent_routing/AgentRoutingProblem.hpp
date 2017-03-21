#ifndef TEMPL_AGENT_ROUTING_AGENT_ROUTING_PROBLEM_HPP
#define TEMPL_AGENT_ROUTING_AGENT_ROUTING_PROBLEM_HPP

#include <vector>
#include "AgentIntegerAttribute.hpp"
#include "Agent.hpp"
#include "AgentType.hpp"

#include "../temporal/TemporalConstraintNetwork.hpp"

namespace templ {
namespace solvers {
namespace agent_routing {

/**
 * \class AgentRoutingProblem
 * \brief the routing problem
 */
class AgentRoutingProblem
{
public:
    AgentRoutingProblem();

    void setTemporalConstraintNetwork(const solvers::temporal::TemporalConstraintNetwork::Ptr& tcn) { mpTemporalConstraintNetwork = tcn; }
    solvers::temporal::TemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() { return mpTemporalConstraintNetwork; }

    const std::vector<AgentType>& getAgentTypes() const { return mAgentTypes;  }
    const std::vector<AgentIntegerAttribute>& getAgentIntegerAttributes() const { return mIntegerAttributes; }
    const std::vector<Agent>& getAgents() const { return mAgents; }

    /**
     * Register an Integer Attribute
     *
     * \throws std::invalid_argument if attribute with same id is already
     * registered
     */
    void addIntegerAttribute(const AgentIntegerAttribute& a);

    /**
     *  Test if an integer attribute with the given id is already registered
     *  \return True if a attribute with given id is registered, False otherwise
     */
    bool hasIntegerAttributeId(uint32_t id) const;

    /**
     * Return the integer attribute with the given id
     */
    const AgentIntegerAttribute& getIntegerAttribute(uint32_t id) const;

    /**
     * Register an agent instance requirement
     */
    void addAgent(const Agent& r);

    /**
     * Register a general agent type
     */
    void addAgentType(const AgentType& type);

    /**
     * Get the actual number of available agents
     */
    uint32_t getNumberOfAgents() const { return mAgents.size(); }

    /**
     * Get the actual number of locations
     */
    uint32_t getNumberOfLocations() const { return mLocations.size(); }

    /**
     * Retrieve all known locations
     */
    const symbols::constants::Location::List& getLocations() const { return mLocations; }

    const solvers::temporal::point_algebra::TimePoint::PtrList& getTimePoints() const { return mTimePoints; }

    /**
     * Create string representation of the overall problem
     */
    std::string toString(uint32_t indent = 0) const;

private:
    std::vector<AgentType> mAgentTypes;
    std::vector<AgentIntegerAttribute> mIntegerAttributes;
    std::vector<Agent> mAgents;

    /// Collected list of locations where agent have to perform tasks
    symbols::constants::Location::List mLocations;
    /// Collected list of timepoint where agent are bound to
    solvers::temporal::point_algebra::TimePoint::PtrList mTimePoints;

    /// Temporal Constraint Network
    solvers::temporal::TemporalConstraintNetwork::Ptr mpTemporalConstraintNetwork;

};

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_PROBLEM_HPP
