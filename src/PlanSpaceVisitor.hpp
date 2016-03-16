#ifndef TEMPL_PLAN_SPACE_VISITOR_HPP
#define TEMPL_PLAN_SPACE_VISITOR_HPP

#include <graph_analysis/BaseGraph.hpp>
#include <graph_analysis/algorithms/DFSVisitor.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/Plan.hpp>

namespace templ {

class MissionPlanner;

class PlanSpaceVisitor : public graph_analysis::algorithms::DFSVisitor
{
public:
    typedef shared_ptr<PlanSpaceVisitor> Ptr;

    PlanSpaceVisitor(const graph_analysis::BaseGraph::Ptr& searchGraph, MissionPlanner* missionPlanner);

    virtual ~PlanSpaceVisitor();

    void discoverVertex(graph_analysis::Vertex::Ptr& vertex);

    const std::vector<Plan>& getPlans() const { return mPlans; }

private:
    graph_analysis::BaseGraph::Ptr mpSearchGraph;
    MissionPlanner* mpMissionPlanner;
    std::vector<Plan> mPlans;

};

} // end namespace templ
#endif // TEMPL_PLAN_SPACE_VISITOR_HPP
