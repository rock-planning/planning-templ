#include "MissionPlanner.hpp"

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <sstream>

#include <base-logging/Logging.hpp>
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
#include <templ/solvers/transshipment/MinCostFlow.hpp>
#include <templ/utils/PathConstructor.hpp>
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
    , mType(BASIC)
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

    uint32_t tcnVariants = 0;
    uint32_t modelDistributionVariants = 0;
    ModelDistribution::SolutionList modelDistributionSolutions;
    RoleDistribution::SolutionList roleDistributionSolutions;
    uint32_t roleDistributionVariants = 0;
    bool stopSearch = false;

    while(!stopSearch && (tcn = nextTemporalConstraintNetwork()) )
    {
        LOG_WARN_S << "TCN generated";
        ++tcnVariants;

        using namespace solvers;
        Mission::Ptr mission(new Mission(mMission));
        graph_analysis::io::GraphIO::write("/tmp/test-tcn-pre.dot", tcn);
        mission->getTemporalConstraintNetwork()->setConsistentNetwork(tcn);
        mission->getTemporalConstraintNetwork()->save("/tmp/test-tcn");
        mission->prepareTimeIntervals();

        csp::ModelDistribution::SearchState modelSearchState(mission);

        bool stopModelSearch = false;
        while(!stopSearch && !stopModelSearch)
        {
            LOG_WARN_S << "MODEL DISTRIBUTION SEARCH";
            csp::ModelDistribution::SearchState modelDistributionState = modelSearchState.next();
            switch(modelDistributionState.getType())
            {
                case csp::ModelDistribution::SearchState::SUCCESS:
                    ++modelDistributionVariants;
                    modelDistributionSolutions.push_back(modelDistributionState.getSolution());
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
            while(!stopSearch && !stopRoleSearch)
            {
                csp::RoleDistribution::SearchState roleDistributionState = roleSearchState.next();
                switch(roleDistributionState.getType())
                {
                    case csp::RoleDistribution::SearchState::SUCCESS:
                        ++roleDistributionVariants;
                        roleDistributionSolutions.push_back(roleDistributionState.getSolution());
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
                transshipment::MinCostFlow minCostFlow(mission, timelines);
                std::vector<transshipment::Flaw> flaws = minCostFlow.run();

                LOG_WARN_S << "Flaws in plan: " << flaws.size();
                if(flaws.empty())
                {
                    Plan plan = renderPlan(mission, minCostFlow.getFlowNetwork().getSpaceTimeNetwork(), timelines);
                    plans.push_back(plan);
                    mission->getLogger()->incrementSessionId();
                    LOG_WARN_S << "Solution found: " << plan.toString();
                }
            }
        }
    }
    return plans;
}


graph_analysis::BaseGraph::Ptr MissionPlanner::nextTemporalConstraintNetwork()
{
    LOG_WARN_S << "NEXT_TEMPORAL";
    graph_analysis::BaseGraph::Ptr solution;
    if(!mpGQReasoner)
    {
        mpGQReasoner = new templ::solvers::GQReasoner("point", mMission.getTemporalConstraintNetwork()->getGraph(), pa::QualitativeTimePointConstraint::Ptr(new pa::QualitativeTimePointConstraint));
        solution = mpGQReasoner->getPrimarySolution();
    } else {
        solution = mpGQReasoner->getNextSolution();

    }
    if(!solution)
    {
        LOG_WARN_S << "No solution exists for temporal constraint network";
    } else {
        LOG_WARN_S << " gqr solution network: " << std::endl
            << mpGQReasoner->getCurrentSolutionString() << std::endl;
    }

    return solution;
}


Plan MissionPlanner::renderPlan(const Mission::Ptr& mission,
        const SpaceTime::Network& spaceTimeNetwork,
        const std::map<Role, csp::RoleTimeline>& timelines,
        const std::string& markerLabel) const
{
    Plan plan(mission, markerLabel);

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

        SpaceTime::Network::tuple_t::Ptr startTuple;
        try {
            startTuple = spaceTimeNetwork.tupleByKeys(location, interval.getFrom());
        } catch(const std::invalid_argument& e)
        {
            throw std::runtime_error("templ::MissionPlanner::renderPlan: failed to find initial tuple for role " + role.toString());
        }

        using namespace graph_analysis::algorithms;
        // use SpaceTime::Network, which contains information on role for each edge
        // after update from the flow graph
        // foreach role -- find starting point and follow path
        PathConstructor::Ptr pathConstructor(new PathConstructor(role));
        Skipper skipper = boost::bind(&PathConstructor::isInvalidTransition, pathConstructor,_1);
        DFS dfs(spaceTimeNetwork.getGraph(), pathConstructor, skipper);
        dfs.run(startTuple);

        std::vector<graph_analysis::Vertex::Ptr> path = pathConstructor->getPath();
        path.insert(path.begin(), startTuple);

        plan.add(role,path);
    }
    return plan;
}

} // end namespace templ
