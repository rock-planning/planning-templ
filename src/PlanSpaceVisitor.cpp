#include "PlanSpaceVisitor.hpp"
#include <templ/PlanningState.hpp>
#include <templ/MissionPlanner.hpp>

namespace templ {

PlanSpaceVisitor::PlanSpaceVisitor(const graph_analysis::BaseGraph::Ptr& searchGraph,
        MissionPlanner* missionPlanner)
    : mpSearchGraph(searchGraph)
    , mpMissionPlanner(missionPlanner)
{}

PlanSpaceVisitor::~PlanSpaceVisitor()
{}

void PlanSpaceVisitor::discoverVertex(graph_analysis::Vertex::Ptr& vertex)
{
//    //LOG_WARN_S << "DISCOVER vertex: " << vertex->toString();
//    using namespace graph_analysis;
//    using namespace graph_analysis::algorithms;
//
//    PlanningState::Ptr planningState = dynamic_pointer_cast<PlanningState>(vertex);
//    // if model distribution not set then we need a new temporal
//    // constraint network
//    LOG_WARN_S << "DISCOVER vertex: " << vertex->toString();
//    if(!planningState->getModelDistribution())
//    {
//        LOG_WARN_S << "FAILED to get model distribution";
//        if(!mpMissionPlanner->nextTemporalConstraintNetwork(planningState))
//        {
//            LOG_WARN_S << "options to modify the temporal constraint network are exhausted: dead end reached";
//            return;
//        } else {
//            LOG_WARN_S << "COMPUTED NEXT TCN";
//        }
//    }
//
//    // If no role distribution is provided we need to get the next model
//    // assignment
//    if(!planningState->getRoleDistribution())
//    {
//        LOG_WARN_S << "FAILD to get role distribution";
//        if(!mpMissionPlanner->nextModelAssignment(planningState))
//        {
//            LOG_WARN_S << "Failed to get model assignment";
//            PlanningState::Ptr planningStateChild = dynamic_pointer_cast<PlanningState>(planningState->clone());
//            planningStateChild->cleanup(PlanningState::MODEL);
//            // Adding a new child
//            Edge::Ptr edge(new Edge());
//            edge->setSourceVertex(planningState);
//            edge->setTargetVertex(planningStateChild);
//            mpSearchGraph->addEdge(edge);
//            return;
//        }
//    }
//
//    LOG_WARN_S << "TRYING TO GET NEXT ROLE ASSIGNMENT";
//    // If we get the next role assignment we can plan
//    if(mpMissionPlanner->nextRoleAssignment(planningState))
//    {
//        LOG_WARN_S << "FOUND role assignment";
//        mpMissionPlanner->computeRoleTimelines(planningState);
//        mpMissionPlanner->computeTemporallyExpandedLocationNetwork(planningState);
//        std::vector<ConstraintViolation> violations = mpMissionPlanner->computeMinCostFlow(planningState);
//
//        if(violations.empty())
//        {
//            mPlans.push_back(mpMissionPlanner->renderPlan());
//            mpMissionPlanner->getLogging().incrementSessionId();
//            LOG_WARN_S << "FOUND PLAN --> this state is a leaf";
//            Vertex::Ptr planningStateChild = planningState->clone();
//            // do not cleanup since role assignment can be continued in
//            // different ways
//            Edge::Ptr edge(new Edge());
//            edge->setSourceVertex(vertex);
//            edge->setTargetVertex(planningStateChild);
//            mpSearchGraph->addEdge(edge);
//            return;
//        } else {
//            using namespace solvers::csp;
//            std::vector<Resolver::Ptr> resolvers = planningState->getResolvers();
//
//            for(uint32_t i = 0; i < resolvers.size(); ++i)
//            {
//                Resolver::Ptr resolver = resolvers[i];
//                LOG_WARN_S << "CLONING current planning state";
//                Vertex::Ptr testV = planningState->clone();
//                assert(testV);
//                PlanningState::Ptr planningStateChild = dynamic_pointer_cast<PlanningState>(testV);
//
//                LOG_WARN_S << "Trying to apply resolver to planning state: " << testV->getClassName()
//                    << " testV " << testV->toString();
//                assert(planningStateChild);
//                assert(planningStateChild.get());
//                try {
//                    resolver->apply(planningStateChild.get());
//                } catch(const std::exception& e)
//                {
//                    LOG_WARN_S << "Could not apply resolver: " << e.what();
//                    continue;
//                }
//
//                Edge::Ptr edge(new Edge());
//                edge->setSourceVertex(vertex);
//                edge->setTargetVertex(planningStateChild);
//                mpSearchGraph->addEdge(edge);
//            }
//            return;
//        }
//    } else {
//
//        LOG_WARN_S << "Failed to find role assignment";
//        PlanningState::Ptr planningStateChild = dynamic_pointer_cast<PlanningState>(planningState->clone());
//        planningStateChild->cleanup(PlanningState::ROLE);
//        // Adding a new child
//        Edge::Ptr edge(new Edge());
//        edge->setSourceVertex(vertex);
//        edge->setTargetVertex(planningStateChild);
//        mpSearchGraph->addEdge(edge);
//    }
}

} // end namespace templ
