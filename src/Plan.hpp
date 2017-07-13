#ifndef TEMPL_PLAN_HPP
#define TEMPL_PLAN_HPP

#include <map>
#include <graph_analysis/Vertex.hpp>
#include <graph_analysis/BaseGraph.hpp>
#include <templ/Mission.hpp>
#include <templ/CapacityLink.hpp>

namespace templ {

/**
 * A represention of a role-based plan
 */
class Plan
{
public:
    /// A Role base plan, consisting of roles and associate path
    typedef std::map<Role, std::vector<graph_analysis::Vertex::Ptr> > RoleBasedPlan;
    typedef std::map<Role, std::vector<std::string> > ActionPlan;

    Plan();

    Plan(const Mission::Ptr& mission, const std::string& label = "");

    void setLabel(const std::string& label) { mLabel = label; }

    const std::string& getLabel() const { return mLabel; }

    Mission::Ptr getMission() const { return mpMission; }

    const RoleBasedPlan& getRoleBasePlan() const { return mRolebasedPlan; }


    /**
     * Add a role-associated-path to the plan
     * \throws if a path is already registered for this role
     */
    void add(const Role& role, const std::vector<graph_analysis::Vertex::Ptr>& path);

    /**
     * Create a string representation of a role plan based on a given mission
     *
     */
    std::string toString(uint32_t indent = 0) const;

    /**
     * Create a string representation of plans
     */
    static std::string toString(const std::vector<Plan>& plans, uint32_t indent = 0);

    /**
     * Create a string representation of and Action plan -- simplified plan
     * using set of predefined actions such as:
     *      move-to (from:  ..., to: ...)
     *      join (provider: ..., from: ..., to: ...)
     */
    static std::string toString(const ActionPlan& plan, uint32_t indent = 0);

    /**
     * Create a string representation of a set of action plans
     */
    static std::string toString(const std::vector<ActionPlan>& plans, uint32_t indent = 0);

    /**
     * Save plan into file of the given name
     * \throws if file cannot be created (e.g. missing parent folder)
     */
    void save( const std::string& filename);

    /**
     * Save all plans in the list into file of the given name
     * \throws std::invalid_argument if file cannot be created (e.g. missing parent folder)
     */
    static void save(const std::vector<Plan>& plans, const std::string& filename);

    /**
     * Save action plans into file of the vigen name
     * \throws std::invalid_argument if file cannot be created
     */
    static void save(const std::vector<ActionPlan>& plans, const std::string& filename);

    /**
     * Write a list of plans to file
     */
    static void saveAsActionPlan(const std::vector<Plan>& plans, const std::string& filename);

    /**
     * Convert plans to actions plans, b
     */
    static std::vector<ActionPlan> toActionPlans(const std::vector<Plan>& plans);


    /*
     * Create an action based plan using set of predefined actions such as:
     *      move-to (from:  ..., to: ...)
     *      join (provider: ..., from: ..., to: ...)
     * \return ActionPlan
     */
    const ActionPlan& getActionPlan() const;

    /**
     * Compute the action plan and graph in one go
     * only perform an update if required
     */
    void computeGraphAndActionPlan() const;

    /**
     * Return the graph representing the plan
     */
    graph_analysis::BaseGraph::Ptr getGraph() const;

private:
    Mission::Ptr mpMission;
    std::string mLabel;
    RoleBasedPlan mRolebasedPlan;

    mutable graph_analysis::BaseGraph::Ptr mpBaseGraph;
    mutable ActionPlan mActionPlan;
    mutable bool mRequiresRefresh;

};

} // end namespace templ
#endif // TEMPL_PLAN_HPP
