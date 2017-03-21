#include "AgentRoutingProblem.hpp"
#include <sstream>
#include <stdexcept>
#include <algorithm>
//#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>


namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace agent_routing {

AgentRoutingProblem::AgentRoutingProblem()
{}

void AgentRoutingProblem::addIntegerAttribute(const AgentIntegerAttribute& a)
{
    if( hasIntegerAttributeId(a.getId()) )
    {
        std::stringstream ss;
        ss << "templ::agent_routing::AgentRoutingProblem::addIntegerAttribute: ";
        ss << " agent integer attribute with id '" << a.getId() << "' is already registered";
        throw std::invalid_argument(ss.str());
    }

    mIntegerAttributes.push_back(a);
    std::sort(mIntegerAttributes.begin(), mIntegerAttributes.end(), [](const AgentIntegerAttribute& a, const AgentIntegerAttribute& b)
           {
               return a.getId() < b.getId();
           });
}

bool AgentRoutingProblem::hasIntegerAttributeId(uint32_t id) const
{
    try {
        getIntegerAttribute(id);
    } catch(const std::invalid_argument& e)
    {
        return false;
    }
    return true;
}

const AgentIntegerAttribute& AgentRoutingProblem::getIntegerAttribute(uint32_t id) const
{
    std::vector<AgentIntegerAttribute>::const_iterator cit;
    cit = std::find_if(mIntegerAttributes.begin(), mIntegerAttributes.end(), [id](const AgentIntegerAttribute& a)->bool
            {
                return a.getId() == id;
            });
    if(cit != mIntegerAttributes.end())
    {
        return *cit;
    } else {
        std::stringstream ss;
        ss << "templ::agent_routing::AgentRoutingProblem: no attribute with id '" << id << "' known";
        throw std::invalid_argument(ss.str() );
    }
}

void AgentRoutingProblem::addAgent(const Agent& r)
{
    mAgents.push_back(r);

    // Locations
    {
        symbols::constants::Location::List::const_iterator lit = r.getLocations().begin();
        for(; lit != r.getLocations().end(); ++lit)
        {
            const symbols::constants::Location& location = *lit;
            symbols::constants::Location::List::const_iterator it;
            it = std::find(mLocations.begin(), mLocations.end(), location);
            if(it == mLocations.end())
            {
                mLocations.push_back(location);
            }
        }
    }

    // Timepoints
    {
        pa::TimePoint::PtrList::const_iterator tit = r.getTimePoints().begin();
        for(; tit != r.getTimePoints().end(); ++tit)
        {
            const pa::TimePoint::Ptr& timepoint = *tit;
            pa::TimePoint::PtrList::const_iterator it;
            it = std::find(mTimePoints.begin(), mTimePoints.end(), timepoint);
            if(it == mTimePoints.end())
            {
                mTimePoints.push_back(timepoint);
            }
        }
    }
}

void AgentRoutingProblem::addAgentType(const AgentType& type)
{
    std::vector<AgentType>::const_iterator ait = std::find_if(mAgentTypes.begin(), mAgentTypes.end(), [type] (const AgentType& agentType)
            {
                return agentType.getTypeId() == type.getTypeId();
            });
    if(ait != mAgentTypes.end())
    {
        std::stringstream ss;
        ss << "templ::agent_routing::AgentRoutingProblem::addAgentType: ";
        ss << "agent type with id '" << type.getTypeId() << "' already registered";
        throw std::invalid_argument(ss.str());
    }
    mAgentTypes.push_back(type);
}


std::string AgentRoutingProblem::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent, ' ');
    ss << hspace << "AgentRoutingProblem" << std::endl;
    ss << hspace << "    attributes:" << std::endl;
    ss << hspace << AgentIntegerAttribute::toString(mIntegerAttributes, indent + 8);
    ss << hspace << "    agent-types:" << std::endl;
    ss << AgentType::toString(mAgentTypes, indent + 8);
    ss << hspace << "    agents:" << std::endl;
    ss << Agent::toString(mAgents, indent + 8);
    ss << hspace << "    temporal constraint network:" << std::endl;
    ss << mpTemporalConstraintNetwork->toString(indent+8);
    return ss.str();
}

} // namespace agent_routing
} // namespace templ
