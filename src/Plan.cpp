#include "Plan.hpp"
#include <sstream>
#include <fstream>
#include <templ/RoleInfoTuple.hpp>
#include <organization_model/facets/Robot.hpp>
#include <base-logging/Logging.hpp>

namespace templ {

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
            ss << hspace << "    - " << (*vit)->toString() << std::endl;
        }
    }
    typedef std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> TimepointList;
    TimepointList timepoints = mpMission->getOrderedTimepoints();
    TimepointList::const_iterator tit = timepoints.begin();
    ss << hspace << "Timepoints:" << std::endl;
    for(; tit != timepoints.end(); ++tit)
    {
        ss << hspace << "    " << (*tit)->toString() << std::endl;
    }

    ss << "Mission: " << mpMission.get() << ", temporal constraint: " << mpMission->getTemporalConstraintNetwork()->getGraph().get();

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

void Plan::save(const std::string& filename)
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        throw std::invalid_argument("templ::Plan::save: could not open file '" + filename + "'");
    }
    file << toString();
    file.close();
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

    mpBaseGraph = graph_analysis::BaseGraph::getInstance();

    if(!mpMission)
    {
        throw std::runtime_error("templ::Plan: plan has no associated mission -- cannot proceed generating action plan");
    }

    organization_model::OrganizationModelAsk organizationModelAsk(mpMission->getOrganizationModel(), mpMission->getAvailableResources(), true);

    ActionPlan actionPlan;

    // handle mobile robots first
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
                if(previousWaypoint)
                {
                    CapacityLink::Ptr link(new CapacityLink(role, capacity));
                    link->setSourceVertex(previousWaypoint);
                    link->setTargetVertex(currentWaypoint);
                    mpBaseGraph->addEdge(link);

                    roleSpecificPlan.push_back("move-to (from: '" + previousWaypoint->toString() + "', to: "
                        "'" + currentWaypoint->toString() + "'");
                }
                previousWaypoint = currentWaypoint;
            }
            actionPlan[role] = roleSpecificPlan;
        }

    }

    // handle only immobile robots and map to CapacityLinks
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
            // TODO: update on usage using facets, e.g. when taking
            // about transport units
            uint32_t capacityUsage = 1;

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
                    std::vector<graph_analysis::Edge::Ptr> edges;
                    try {
                        edges = mpBaseGraph->getEdges(previousWaypoint, currentWaypoint);
                    } catch(const std::runtime_error& e)
                    {
                        // if edge is not part of the graph then the item
                        // remains at the current location
                        LOG_WARN_S << "Looks like the items remains: " << previousWaypoint->toString() << " -- " << currentWaypoint->toString() << " -- nothing to do";
                    }

                    if(edges.empty())
                    {
                        //throw std::runtime_error("templ::Plan::getActionPlan: no CapacityLink available to route immobile unit -- this should never happen: please inform the developer");
                        // items remains at the same location (or has to be
                        // dropped
                        capacityProvider = Role();
                    } else if(edges.size() == 1)
                    {
                        CapacityLink::Ptr link = dynamic_pointer_cast<CapacityLink>(edges[0]);
                        link->addUser(role, capacityUsage);
                        // prefer remaining with the same provider
                        capacityProvider = link->getProvider();
                    } else {
                        CapacityLink::Ptr bestLink;
                        std::vector<graph_analysis::Edge::Ptr>::const_iterator eit = edges.begin();
                        for(; eit != edges.end(); ++eit)
                        {
                            CapacityLink::Ptr link = dynamic_pointer_cast<CapacityLink>(*eit);
                            uint32_t remainingCapacity = link->getRemainingCapacity();
                            if(link->getProvider() == capacityProvider)
                            {
                                if(remainingCapacity >= capacityUsage)
                                {
                                    bestLink = link;
                                    break;
                                } else {
                                    LOG_WARN_S << "Found previous capacity provider, but there is no capacity left";
                                }
                            }

                            if(bestLink)
                            {
                                if(remainingCapacity > bestLink->getRemainingCapacity())
                                {
                                    bestLink = link;
                                }
                            } else {
                                bestLink = link;
                            }
                        }
                        bestLink->addUser(role, capacityUsage);
                        capacityProvider = bestLink->getProvider();
                    }

                    if(previousCapacityProvider != capacityProvider)
                    {
                        if(capacityProvider == Role())
                        {
                            roleSpecificPlan.push_back("join (provider: '" + capacityProvider.toString() + "' from: '" + previousWaypoint->toString() + "', to"
                                "'" + currentWaypoint->toString() + "'");
                        } else {
                            roleSpecificPlan.push_back("join (provider: '" + capacityProvider.toString() + "' from: '" + previousWaypoint->toString() + "', to"
                                "'" + currentWaypoint->toString() + "'");
                        }
                    } else {
                        LOG_WARN_S << "Previous provider: '" << previousCapacityProvider.toString() << " --vs-- " << capacityProvider.toString();

                    }
                }
                previousCapacityProvider = capacityProvider;
                previousWaypoint = currentWaypoint;
            }
            actionPlan[role] = roleSpecificPlan;
        }
    }

    mRequiresRefresh = false;
}

} // end namespace templ
