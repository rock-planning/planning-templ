#include "AgentRoutingProblem.hpp"
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include "../temporal/QualitativeTemporalConstraintNetwork.hpp"


namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace solvers {
namespace agent_routing {

AgentRoutingProblem::AgentRoutingProblem()
{}

void AgentRoutingProblem::finalize()
{
    if(mpTemporalConstraintNetwork)
    {
        mpTemporalConstraintNetwork->sort(mTimePoints);
    }

    for(size_t i = 0; i < mAgentTypes.size(); ++i)
    {
        if(mAgentTypes[i].getTypeId() !=  i)
        {
            throw std::runtime_error("templ::solvers::agent_routing::AgentRoutingProblem: agent type violates index == typeId assumption");
        }
    }

    for(size_t i = 0; i < mIntegerAttributes.size(); ++i)
    {
        if(mIntegerAttributes[i].getId() != i)
        {
            throw std::runtime_error("templ::solvers::agent_routing::AgentRoutingProblem: agent integer attribute violates index == id assumption");
        }
    }
    for(size_t i = 0; i < mAgents.size(); ++i)
    {
        if(mAgents[i].getAgentId() != i)
        {
            throw std::runtime_error("templ::solvers::agent_routing::AgentRoutingProblem: agent violates index == agent id assumption");
        }
    }
}

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

const AgentIntegerAttribute& AgentRoutingProblem::getIntegerAttribute(uint32_t id, AgentTypeId typeId) const
{
    return mAgentTypes[typeId].getIntegerAttribute(id);
}

void AgentRoutingProblem::addAgent(const Agent& r)
{
    mAgents.push_back(r);

    // Mobility attribute check
    uint32_t mobilityAttributeId = getMobilityAttributeId();
    AgentIntegerAttribute attribute = getIntegerAttribute(mobilityAttributeId, r.getAgentType());

    if(attribute.getValue() != 0)
    {
        mMobileAgentIds.push_back(r.getAgentId());
    } else {
        mCommodityAgentIds.push_back(r.getAgentId());
    }

    // Locations
    {
        using namespace symbols::constants;
        Location::List::const_iterator lit = r.getLocations().begin();
        for(; lit != r.getLocations().end(); ++lit)
        {
            const Location& location = *lit;
            Location::PtrList::const_iterator it;
            it = std::find_if(mLocations.begin(), mLocations.end(), [location](const Location::Ptr& other)->bool
                    {
                        return location == *other.get();
                    });

            if(it == mLocations.end())
            {
                mLocations.push_back(Location::Ptr(new Location(location)));
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

uint32_t AgentRoutingProblem::getMobilityAttributeId() const
{
    std::map<AttributeName, uint32_t>::const_iterator ait = mAttributeMap.find("mobile");
    if(ait != mAttributeMap.end())
    {
        return ait->second;
    }

    // Lazily initalize attributes hash
    AgentIntegerAttribute::List::const_iterator cit;
    cit = std::find_if(mIntegerAttributes.begin(), mIntegerAttributes.end(), [](const AgentIntegerAttribute& a)
            {
                return a.getLabel() == "mobile";
            });
    if(cit == mIntegerAttributes.end())
    {
        throw std::runtime_error("templ::solvers::agent_routing::AgentRoutingProblem: no attribute 'mobile' is known");
    } else {
        uint32_t id = cit->getId();
        mAttributeMap[cit->getLabel()] = id;
        return id;
    }
}

} // end namespace agent_routing
} // end namepspace solvers
} // end namespace templ
