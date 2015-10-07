#include <templ/io/MissionReader.hpp>
//#include <templ/MissionPlanner.hpp>

#include <templ/solvers/csp/RoleDistribution.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>

#include <templ/LocationTimepointTuple.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <limits>

using namespace templ;

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        printf("usage: %s <mission> <organization-model>\n", argv[0]);
        exit(-1);
    }

    std::string missionFilename = argv[1];
    std::string organizationModelFilename = argv[2];

    using namespace templ;
    Mission mission = io::MissionReader::fromFile(missionFilename);
    printf("%s\n",mission.toString().c_str());

    using namespace organization_model;
    OrganizationModel::Ptr organizationModel = OrganizationModel::getInstance(organizationModelFilename);
    mission.setOrganizationModel(organizationModel);

    mission.prepare();

    using namespace solvers::csp;
    std::vector<ModelDistribution::Solution> solutions = ModelDistribution::solve(mission);
    if(!solutions.empty())
    {
        std::cout << "Solutions for model distribution found: " << solutions << std::endl;
    } else {
        throw std::runtime_error("No feasible model distribution");
    }

    // Check role distribution after model distribution, i.e., using one
    // solution of the model distribution to find a role assignment
    std::vector<RoleDistribution::Solution> roleSolutions = RoleDistribution::solve(mission, solutions[0]);
    if(!roleSolutions.empty())
    {
        std::cout << "Solutions for role distribution: " << roleSolutions << std::endl;
    } else {
        throw std::runtime_error("No feasible role distribution");
    }


    std::cout << "-- BEGIN TIMELINES --------" << std::endl;

    // Compute the timelines per role for one role solution
    std::map<Role, RoleTimeline> timelines = RoleTimeline::computeTimelines(mission, roleSolutions[0]);
    {
        std::map<Role, RoleTimeline>::iterator it = timelines.begin();
        for(; it != timelines.end(); ++it)
        {
            const RoleTimeline& timeline = it->second;
            std::cout << timeline.toString() << std::endl;
        }
    }

    std::cout << "-- BEGIN TEMPORALLY-EXPANDED LOCATION NETWORK -------" << std::endl;

    // basic graph creating can be done even earlier -- setting of edges is
    // required according to the selected robot assignments
    
    // Create temporally-expanded location network
    // timepoints -- locations
    //
    // foreach location l in L
    //     foreach timepoint t in T
    //         create node (t,l)
    //         add to t indexed sorted location
    //         add to l indexed sorted timepoints
    // 
    // need to known all timepoints: from mission.getTimepoints()
    // --> all vertices
    {
        namespace pa = templ::solvers::temporal::point_algebra;
        namespace co = templ::symbols::constants;

        using namespace graph_analysis;
        std::vector<pa::TimePoint::Ptr> timepoints = mission.getTimepoints();
        std::sort(timepoints.begin(), timepoints.end());

        std::vector<co::Location::Ptr> locations = mission.getLocations();

        BaseGraph::Ptr spaceTimeGraph = BaseGraph::getInstance();

        std::vector<co::Location::Ptr>::const_iterator lit = locations.begin();
        for(; lit != locations.end(); ++lit)
        {
            boost::shared_ptr<LocationTimepointTuple> lastTuple;

            std::vector<pa::TimePoint::Ptr>::const_iterator tit = timepoints.begin();
            for(; tit != timepoints.end(); ++tit)
            {
                boost::shared_ptr<LocationTimepointTuple> ltTuplePtr(new LocationTimepointTuple(*lit, *tit));
                spaceTimeGraph->addVertex(ltTuplePtr);

                if(lastTuple)
                {
                    WeightedEdge::Ptr edge(new WeightedEdge(lastTuple, ltTuplePtr, std::numeric_limits<WeightedEdge::value_t>::max()));
                    spaceTimeGraph->addEdge(edge);
                }
                lastTuple = ltTuplePtr;
            }
        }

        std::string filename = "/tmp/space-time-graph.dot";
        graph_analysis::io::GraphIO::write(filename, spaceTimeGraph);
        std::cout << "Written temporally expanded graph to: " << filename << std::endl;
        std::cout << "(e.g. view with 'xdot " << filename << "'" << ")" << std::endl;
    }

    // Per Role --> add capacities (in terms of capability of carrying a
    // payload)


    // Analyse the cost of the planning approach



//    MissionPlanner missionPlanner(mission);
//    CandidateMissions missions = missionPlanner.solve();

    return 0;
}
