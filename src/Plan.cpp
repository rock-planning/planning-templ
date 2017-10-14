#include "Plan.hpp"
#include <sstream>
#include <fstream>
#include <organization_model/facets/Robot.hpp>
#include <base-logging/Logging.hpp>
#include "SpaceTime.hpp"
#include "RoleInfoTuple.hpp"

namespace templ {

Plan::Plan()
    : mpMission()
    , mLabel("empty plan")
    , mRequiresRefresh(false)
{}

Plan::Plan(const Mission::Ptr& mission, const std::string& label)
    : mpMission(mission)
    , mLabel(label)
    /// Set refresh to true to allow lazy initialization of datastructures
    , mRequiresRefresh(true)
{}

void Plan::add(const Role& role, const std::vector<graph_analysis::Vertex::Ptr>& path)
{
    if(mRolebasedPlan.find(role) != mRolebasedPlan.end())
    {
        throw std::invalid_argument("templ::Plan::add: role '" + role.toString() + "' already added to plan");
    }
    mRolebasedPlan[role] = path;
    mRequiresRefresh = true;
}


std::string Plan::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');

    RoleBasedPlan::const_iterator cit = mRolebasedPlan.begin();
    ss << hspace << "plan: " << mLabel << std::endl;
    for(; cit != mRolebasedPlan.end(); ++cit)
    {
        ss << hspace << "- role: " << (cit->first).toString() << std::endl;
        std::vector<graph_analysis::Vertex::Ptr>::const_iterator vit = (cit->second).begin();
        for(; vit != (cit->second).end(); ++vit)
        {
            ss << (*vit)->toString(indent + 4) << std::endl;
        }
    }
    typedef std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> TimepointList;
    TimepointList timepoints = mpMission->getTimepoints();
    TimepointList::const_iterator tit = timepoints.begin();
    ss << hspace << "Timepoints:" << std::endl;
    for(; tit != timepoints.end(); ++tit)
    {
        ss << hspace << "    " << (*tit)->toString() << std::endl;
    }

    //ss << "Mission: " << mpMission.get() << ", temporal constraint: " << mpMission->getTemporalConstraintNetwork()->getGraph().get();

    return ss.str();
}

std::string Plan::toString(const std::vector<Plan>& plans, uint32_t indent)
{
    std::stringstream ss;
    std::vector<Plan>::const_iterator cit = plans.begin();
    for(; cit != plans.end(); ++cit)
    {
        ss << cit->toString(indent);
    }
    return ss.str();
}

void Plan::save(const std::string& filename) const
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        throw std::invalid_argument("templ::Plan::save: could not open file '" + filename + "'");
    }
    file << toString();
    file.close();
}

void Plan::saveGraph(const std::string& filename) const
{
    graph_analysis::io::GraphIO::write(filename, mpBaseGraph);
}

std::string Plan::toString(const ActionPlan& plan, uint32_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;

    ActionPlan::const_iterator ait = plan.begin();
    for(; ait != plan.end(); ++ait)
    {
        const Role& role = ait->first;
        ss << hspace << role.toString() << ":" << std::endl;

        const std::vector<std::string>& actions = ait->second;

        std::vector<std::string>::const_iterator sit = actions.begin();
        for(; sit != actions.end(); ++sit)
        {
            ss << hspace << "    " << *sit << std::endl;
        }
    }
    return ss.str();
}

std::string Plan::toString(const std::vector<ActionPlan>& plans, uint32_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;

    std::vector<ActionPlan>::const_iterator cit = plans.begin();
    for(; cit != plans.end(); ++cit)
    {
        ss << "plan:" << std::endl;
        const ActionPlan& actionPlan = *cit;
        ss << hspace << Plan::toString(actionPlan, indent + 4);
    }

    return ss.str();
}

void Plan::save(const std::vector<Plan>& plans, const std::string& filename)
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        throw std::invalid_argument("templ::Plan::save: could not open file '" + filename + "'");
    }
    file << toString(plans);
    file.close();
}

std::vector<Plan::ActionPlan> Plan::toActionPlans(const std::vector<Plan>& plans)
{
    std::vector<ActionPlan> actionPlans;
    std::vector<Plan>::const_iterator pit = plans.begin();
    for(; pit != plans.end(); ++pit)
    {
        Plan plan = *pit;
        ActionPlan actionPlan = plan.getActionPlan();
        actionPlans.push_back(actionPlan);
    }
    return actionPlans;
}

void Plan::save(const std::vector<Plan::ActionPlan>& plans, const std::string& filename)
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        throw std::invalid_argument("templ::Plan::save: could not open file '" + filename + "'");
    }

    std::vector<ActionPlan>::const_iterator pit = plans.begin();
    for(; pit != plans.end(); ++pit)
    {
        ActionPlan actionPlan = *pit;
        file << toString(actionPlan);
    }
    file.close();
}

void Plan::saveAsActionPlan(const std::vector<Plan>& plans, const std::string& filename)
{
    std::vector<ActionPlan> actionPlans = toActionPlans(plans);
    save(actionPlans, filename);
}

const Plan::ActionPlan& Plan::getActionPlan() const
{
    computeGraphAndActionPlan();
    return mActionPlan;
}

graph_analysis::BaseGraph::Ptr Plan::getGraph() const
{
    computeGraphAndActionPlan();
    return mpBaseGraph;
}

void Plan::computeGraphAndActionPlan() const
{
    if(!mRequiresRefresh)
    {
        return;
    }

    computeGraph();
    computeActionPlan(); //ActionPlan actionPlan;

    mRequiresRefresh = false;
}

void Plan::computeActionPlan() const
{
    ActionPlan actionPlan;
}

void Plan::computeGraph() const
{
    mpBaseGraph = graph_analysis::BaseGraph::getInstance();

    if(!mpMission)
    {
        throw std::runtime_error("templ::Plan: plan has no associated mission -- cannot proceed generating action plan");
    }

    organization_model::OrganizationModelAsk organizationModelAsk(mpMission->getOrganizationModel(), mpMission->getAvailableResources(), true);

    // handle mobile robots first
    // for each mobile system we add capacity links
    {
        RoleBasedPlan::const_iterator cit = mRolebasedPlan.begin();
        for(; cit != mRolebasedPlan.end(); ++cit)
        {
            const Role& role = cit->first;

            // getTransportSystem
            organization_model::facets::Robot robot(role.getModel(), organizationModelAsk);
            if(!robot.isMobile())
            {
                continue;
            }

            std::vector<std::string> roleSpecificPlan;
            uint32_t capacity = robot.getPayloadTransportCapacity();

            const RoleBasedPlan::mapped_type& plan = cit->second;
            RoleBasedPlan::mapped_type::const_iterator pit = plan.begin();
            graph_analysis::Vertex::Ptr previousWaypoint;
            for(; pit != plan.end(); ++pit)
            {
                graph_analysis::Vertex::Ptr currentWaypoint = *pit;
                if(!previousWaypoint)
                {
                    previousWaypoint = currentWaypoint;
                    continue;
                }

                std::vector<graph_analysis::Edge::Ptr> edges;
                try {
                    edges = mpBaseGraph->getEdges(previousWaypoint, currentWaypoint);
                } catch(const std::runtime_error& e)
                {
                    // not link created yet
                }
                if(edges.empty())
                {
                    CapacityLink::Ptr link(new CapacityLink(role, capacity));
                    link->setSourceVertex(previousWaypoint);
                    link->setTargetVertex(currentWaypoint);
                    mpBaseGraph->addEdge(link);
                    LOG_DEBUG_S << "Adding capacity link: for " << role.toString() << " from " << previousWaypoint->toString()
                        << " to " << currentWaypoint->toString();
                    roleSpecificPlan.push_back("move-to (from: '" + previousWaypoint->toString() + "', to: "
                        "'" + currentWaypoint->toString() + "'");
                } else {
                    assert(edges.size() == 1);
                    CapacityLink::Ptr link = dynamic_pointer_cast<CapacityLink>(edges[0]);
                    link->addProvider(role, capacity);
                    LOG_DEBUG_S << "Adding provider to capacity link: for " << role.toString() << " from " << previousWaypoint->toString()
                        << " to " << currentWaypoint->toString();
                    roleSpecificPlan.push_back("move-to (from: '" + previousWaypoint->toString() + "', to: "
                        "'" + currentWaypoint->toString() + "'");
                }
                previousWaypoint = currentWaypoint;
            }
        }
    } // end handling mobile roles

    // handle only immobile robots and map to CapacityLinks
    Role locationTransitionRole = CapacityLink::getLocalTransitionRole();
    {
        RoleBasedPlan::const_iterator cit = mRolebasedPlan.begin();
        for(; cit != mRolebasedPlan.end(); ++cit)
        {
            const Role& role = cit->first;
            const RoleBasedPlan::mapped_type& plan = cit->second;

            organization_model::facets::Robot robot(role.getModel(), organizationModelAsk);
            if(robot.isMobile())
            {
                continue;
            }

            // Extract actual demand for the immobile payload
            int32_t demand = robot.getPayloadTransportSupplyDemand();
            if(demand >= 0)
            {
                throw std::invalid_argument("templ::Plan::computeGraph: expected demand (negative value for supplyDeman) for immobile role '" + role.toString() + "' but was the value was positive");
            }
            uint32_t capacityUsage = abs(demand);

            std::vector<std::string> roleSpecificPlan;
            graph_analysis::Vertex::Ptr previousWaypoint;
            Role previousCapacityProvider;
            Role capacityProvider;

            RoleBasedPlan::mapped_type::const_iterator pit = plan.begin();
            for(; pit != plan.end(); ++pit)
            {
                graph_analysis::Vertex::Ptr currentWaypoint = *pit;
                if(previousWaypoint)
                {
                    CapacityLink::Ptr capacityLink;
                    std::vector<graph_analysis::Edge::Ptr> edges;
                    try {
                        edges = mpBaseGraph->getEdges(previousWaypoint, currentWaypoint);
                    } catch(const std::runtime_error& e)
                    {
                        LOG_INFO_S << "Looks like the items remains: " << previousWaypoint->toString() << " -- " << currentWaypoint->toString();
                    }

                    if(isLocalTransition(previousWaypoint, currentWaypoint))
                    {
                        if(edges.empty())
                        {
                                // Lazily creating the link of a local transition
                                CapacityLink::Ptr localLink( new CapacityLink(locationTransitionRole, std::numeric_limits<uint32_t>::max()));
                                localLink->setSourceVertex(previousWaypoint);
                                localLink->setTargetVertex(currentWaypoint);
                                mpBaseGraph->addEdge(localLink);

                                capacityLink = localLink;
                        } else {
                            try {
                                capacityLink = dynamic_pointer_cast<CapacityLink>(edges[0]);
                                capacityLink->addProvider(locationTransitionRole, std::numeric_limits<uint32_t>::max());
                            } catch(const std::invalid_argument& e)
                            {
                                // ignore if local transition role has already
                                // been added
                            }
                        }
                    } else if(edges.empty())
                    {
                        throw std::runtime_error("templ::Plan::computeGraphAndActionPlan: item can not be routed, neither a transport provider is available, nor is this a local transition");
                    }else {
                        assert(edges.size() == 1);
                        capacityLink = dynamic_pointer_cast<CapacityLink>(edges[0]);
                    }

                    capacityLink->addConsumer(role, capacityUsage);
                }
                previousWaypoint = currentWaypoint;
            }
        } // end role based plan
    }
}

bool Plan::isLocalTransition(const graph_analysis::Vertex::Ptr& v0, const graph_analysis::Vertex::Ptr& v1) const
{
    SpaceTime::SpaceTimeTuple::Ptr tuple0 = dynamic_pointer_cast<SpaceTime::SpaceTimeTuple>(v0);
    SpaceTime::SpaceTimeTuple::Ptr tuple1 = dynamic_pointer_cast<SpaceTime::SpaceTimeTuple>(v1);

    assert(tuple0);
    assert(tuple1);

    return tuple0->first() == tuple1->first();
}

} // end namespace templ
