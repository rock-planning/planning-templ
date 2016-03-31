#ifndef TEMPL_PLAN_HPP
#define TEMPL_PLAN_HPP

#include <map>
#include <graph_analysis/Vertex.hpp>
#include <graph_analysis/BaseGraph.hpp>
#include <templ/Mission.hpp>
#include <templ/CapacityLink.hpp>

namespace templ {

class Plan
{
public:
    typedef std::map<Role, std::vector<graph_analysis::Vertex::Ptr> > RoleBasedPlan;
    typedef std::map<Role, std::vector<std::string> > ActionPlan;

    Plan(const Mission::Ptr& mission, const std::string& label = "");

    void setLabel(const std::string& label) { mLabel = label; }
    const std::string& getLabel() const { return mLabel; }

    /**
     * Add a role associated path to the plan
     * \throws if a path is already registered for this role
     */
    void add(const Role& role, const std::vector<graph_analysis::Vertex::Ptr>& path);

    std::string toString(uint32_t indent = 0) const;

    static std::string toString(const std::vector<Plan>& plans, uint32_t indent = 0);

    static std::string toString(const ActionPlan& plan, uint32_t indent = 0);
    static std::string toString(const std::vector<ActionPlan>& plans, uint32_t indent = 0);

    /**
     * Save plan into file of the given name
     * \throws if file cannot be created (e.g. missing parent folder)
     */
    void save( const std::string& filename);

    /**
     * Save all plans in the list into file of the given name
     * \throws if file cannot be created (e.g. missing parent folder)
     */
    static void save(const std::vector<Plan>& plans, const std::string& filename);

    static void saveAsActionPlan(const std::vector<Plan>& plans, const Mission& mission, const std::string& filename);

    ActionPlan getActionPlan(const Mission& mission);


private:
    Mission::Ptr mpMission;
    std::string mLabel;
    RoleBasedPlan mRolebasedPlan;

    graph_analysis::BaseGraph::Ptr mpBaseGraph;
};

} // end namespace templ
#endif // TEMPL_PLAN_HPP
