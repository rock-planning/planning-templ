#include "MissionPlanner.hpp"

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <sstream>

#include <base/Logging.hpp>
#include <base/Time.hpp>

#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/SharedPtr.hpp>
#include <limits>

#include <organization_model/vocabularies/OM.hpp>
#include <organization_model/facets/Robot.hpp>

#include <templ/solvers/GQReasoner.hpp>
#include <templ/solvers/csp/ModelDistribution.hpp>
#include <templ/solvers/csp/RoleDistribution.hpp>
#include <templ/solvers/transshipment/TransportNetwork.hpp>
#include <templ/solvers/transshipment/MinCostFlow.hpp>
#include <templ/PathConstructor.hpp>
#include <templ/Plan.hpp>

using namespace organization_model;
using namespace templ::solvers;
using namespace templ::solvers::csp;
namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {

MissionPlanner::MissionPlanner(const Mission& mission)
    : mMission(mission)
    , mpGQReasoner(0)
    , mpLogger(mission.getLogger())
{
    if(!mMission.getTimeIntervals().empty())
    {
        throw std::invalid_argument("templ::MissionPlanner: cannot construct MissionPlanner object"
                "from unprepared mission -- no time intervals set");
    }
}

MissionPlanner::~MissionPlanner()
{
    delete mpGQReasoner;
}

std::vector<Plan> MissionPlanner::execute(uint32_t)
{
    std::vector<Plan> plans;

    graph_analysis::BaseGraph::Ptr tcn;

    while( (tcn = nextTemporalConstraintNetwork()) )
    {
        LOG_WARN_S << "TCN generated";

        using namespace solvers;
        Mission::Ptr mission(new Mission(mMission));
        mission->getTemporalConstraintNetwork()->setConsistentNetwork(tcn);
        mission->getTemporalConstraintNetwork()->save("/tmp/test-tcn");
        mission->prepareTimeIntervals();

        csp::ModelDistribution::SearchState modelSearchState(mission);

        bool stopModelSearch = false;
        while(!stopModelSearch)
        {
            LOG_WARN_S << "MODEL DISTRIBUTION SEARCH";
            csp::ModelDistribution::SearchState modelDistributionState = modelSearchState.next();
            switch(modelDistributionState.getType())
            {
                case csp::ModelDistribution::SearchState::SUCCESS:
                    LOG_WARN_S << "MODEL DISTRIBUTION SEARCH: success";
                    break;
                case csp::ModelDistribution::SearchState::FAILED:
                    LOG_WARN_S << "MODEL DISTRIBUTION SEARCH: failed";
                    stopModelSearch = true;
                    continue;
                case csp::ModelDistribution::SearchState::OPEN:
                    assert(false);
            }

            LOG_WARN_S << "ROLE DISTRIBUTION SEARCH";
            csp::RoleDistribution::SearchState roleSearchState(modelDistributionState);
            bool stopRoleSearch = false;
            while(!stopRoleSearch)
            {
                csp::RoleDistribution::SearchState roleDistributionState = roleSearchState.next();
                switch(roleDistributionState.getType())
                {
                    case csp::RoleDistribution::SearchState::SUCCESS:
                        LOG_WARN_S << "ROLE DISTRIBUTION SEARCH: success";
                        break;
                    case csp::RoleDistribution::SearchState::FAILED:
                        LOG_WARN_S << "ROLE DISTRIBUTION SEARCH: failed";
                        stopRoleSearch = true;
                        continue;
                    case csp::RoleDistribution::SearchState::OPEN:
                        assert(false);
                }

                LOG_WARN_S << "FLOW OPTIMIZATION";
                using namespace solvers;
                std::map<Role, csp::RoleTimeline> timelines =  RoleTimeline::computeTimelines(*mission.get(), roleDistributionState.getSolution());
                transshipment::TransportNetwork transportNetwork(mission, timelines);
                transshipment::MinCostFlow minCostFlow(mission, timelines, transportNetwork);
                std::vector<transshipment::Flaw> flaws = minCostFlow.run();

                LOG_WARN_S << "Flaws in plan: " << flaws.size();
                if(flaws.empty())
                {
                    LOG_WARN_S << "Solution found";
                    Plan plan = renderPlan(&transportNetwork.getSpaceTimeNetwork(), timelines);
                    plans.push_back(plan);
                    return plans;
                }
            }
        }
    }
    return plans;
}


graph_analysis::BaseGraph::Ptr MissionPlanner::nextTemporalConstraintNetwork()
{
    graph_analysis::BaseGraph::Ptr solution;
    if(!mpGQReasoner)
    {
        mpGQReasoner = new templ::solvers::GQReasoner("point", mMission.getTemporalConstraintNetwork()->getGraph(), pa::QualitativeTimePointConstraint::Ptr(new pa::QualitativeTimePointConstraint));
        solution = mpGQReasoner->getPrimarySolution();
    } else {
        solution = mpGQReasoner->getNextSolution();

        LOG_DEBUG_S << " gqr solution network: " << std::endl
            << mpGQReasoner->getCurrentSolutionString() << std::endl;
    }

    if(!solution)
    {
        LOG_DEBUG_S << "No solution exists for temporal constraint network";
    }
    return solution;

    //else {
    //    state->getTemporalConstraintNetwork()->setGraph(solution);
    //    prepareTemporalConstraintNetwork(state);
    //    return true;
    //}
}


//ModelDistribution::SearchState MissionPlanner::nextModelDistributionSearch(ModelDistribution::SearchState currentSearchState)
//{
//    } else {
//
//    if(solvedDistribution)
//    {
//        state->setModelDistributionSolution(solvedDistribution->getSolution());
//        delete solvedDistribution;
//        LOG_WARN_S << "Found model assignment: " << state->getModelDistributionSolution();
//        return true;
//    } else {
//        state->cleanup(PlanningState::MODEL);
//        return false;
//    }
//}

Plan MissionPlanner::renderPlan(SpaceTimeNetwork* spaceTimeNetwork,
        const std::map<Role, csp::RoleTimeline>& timelines,
        const std::string& markerLabel) const
{
    Plan plan(markerLabel);

    std::map<Role, RoleTimeline>::const_iterator it = timelines.begin();
    for(; it != timelines.end(); ++it)
    {
        const Role& role = it->first;
        csp::RoleTimeline roleTimeline = it->second;
        roleTimeline.sortByTime();

        // Get first space/location tuple
        const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
        symbols::constants::Location::Ptr location = roleTimeline.getLocation(ftrs.front());
        solvers::temporal::Interval interval = roleTimeline.getInterval(ftrs.front());

        SpaceTimeNetwork::tuple_t::Ptr startTuple;
        try {
            startTuple = spaceTimeNetwork->tupleByKeys(location, interval.getFrom());
        } catch(const std::invalid_argument& e)
        {
            throw std::runtime_error("templ::MissionPlanner::renderPlan: failed to find initial tuple for role " + role.toString());
        }

        using namespace graph_analysis::algorithms;
        // use SpaceTimeNetwork, which contains information on role for each edge
        // after update from the flow graph
        // foreach role -- find starting point and follow path
        PathConstructor::Ptr pathConstructor(new PathConstructor(role));
        boost::function1<bool, graph_analysis::Edge::Ptr> skipper = boost::bind(&PathConstructor::invalidTransition, pathConstructor,_1);
        DFS dfs(spaceTimeNetwork->getGraph(), pathConstructor, skipper);
        dfs.run(startTuple);

        plan.add(role, pathConstructor->getPath());
    }
    return plan;
}

} // end namespace templ
