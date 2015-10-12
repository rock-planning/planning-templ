#include <templ/io/MissionReader.hpp>
//#include <templ/MissionPlanner.hpp>

#include <templ/solvers/csp/RoleDistribution.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>

#include <templ/LocationTimepointTuple.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <limits>

#include <owlapi/Vocabulary.hpp>
#include <organization_model/facets/Robot.hpp>

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
        pa::TimePointComparator tpc(mission.getTemporalConstraintNetwork());
        std::sort(timepoints.begin(), timepoints.end(), [&tpc](const pa::TimePoint::Ptr& a, const pa::TimePoint::Ptr& b)
                {
                    if(a == b)
                    {
                        return false;
                    }

                    return tpc.lessThan(a,b);
                }
        );

        std::vector<co::Location::Ptr> locations = mission.getLocations();

        BaseGraph::Ptr spaceTimeGraph = BaseGraph::getInstance();
        typedef std::pair< co::Location::Ptr, pa::TimePoint::Ptr> LocationTimePointPair;
        std::map< LocationTimePointPair, LocationTimepointTuple::Ptr > tupleMap;

        std::vector<co::Location::Ptr>::const_iterator lit = locations.begin();
        for(; lit != locations.end(); ++lit)
        {
            LocationTimepointTuple::Ptr lastTuple;

            std::vector<pa::TimePoint::Ptr>::const_iterator tit = timepoints.begin();
            for(; tit != timepoints.end(); ++tit)
            {
                LocationTimepointTuple::Ptr ltTuplePtr(new LocationTimepointTuple(*lit, *tit));
                spaceTimeGraph->addVertex(ltTuplePtr);
                tupleMap[ LocationTimePointPair(*lit, *tit) ] = ltTuplePtr;

                if(lastTuple)
                {
                    WeightedEdge::Ptr edge(new WeightedEdge(lastTuple, ltTuplePtr, std::numeric_limits<WeightedEdge::value_t>::max()));
                    spaceTimeGraph->addEdge(edge);
                }
                lastTuple = ltTuplePtr;
            }
        }

        {
            std::string filename = "/tmp/space-time-graph.dot";
            graph_analysis::io::GraphIO::write(filename, spaceTimeGraph);
            std::cout << "Written temporally expanded graph to: " << filename << std::endl;
            std::cout << "(e.g. view with 'xdot " << filename << "'" << ")" << std::endl;
        }
        {
            std::string filename = "/tmp/mission-temporally-constrained-network.dot";
            graph_analysis::io::GraphIO::write(filename, mission.getTemporalConstraintNetwork()->getGraph());
            std::cout << "Written temporal constraint network to: " << filename << std::endl;
            std::cout << "(e.g. view with 'xdot " << filename << "'" << ")" << std::endl;
        }

        // Per Role --> add capacities (in terms of capability of carrying a
        // payload)
        //std::map<Role, RoleTimeline> timelines
        std::map<Role, RoleTimeline>::const_iterator rit = timelines.begin();
        for(; rit != timelines.end(); ++rit)
        {
            // infer connections from timeline
            // sequentially ordered timeline
            // locations and timeline
            // connection from (l0, i0_end) ---> (l1, i1_start)
            //
            const Role& role = rit->first;
            const RoleTimeline& roleTimeline = rit->second;

            // Check if this item is a payload -- WARNING: this is domain specific
            owlapi::model::OWLOntologyAsk ask(organizationModel->ontology());
            owlapi::model::IRI payloadClass = owlapi::vocabulary::OM::resolve("Payload");
            if(payloadClass == role.getModel() || ask.isSubClassOf(role.getModel(), owlapi::vocabulary::OM::resolve("Payload")))
            {
                std::cout << "Delay handling of payload" << std::endl;
                continue;
            } else {
                std::cout << "HANDLING: " << role.getModel() << " since it is not a " << payloadClass << std::endl;
            }

            // infer capacity from role -- when robot is mobile / has
            // transportCapacity
            organization_model::facets::Robot robot(role.getModel(), organizationModel);

            uint32_t capacity = robot.getPayloadTransportCapacity();
            std::cout << "Role: " << role.toString() << std::endl
                << "    transport capacity: " << capacity << std::endl;


            namespace pa = templ::solvers::temporal::point_algebra;
            pa::TimePoint::Ptr prevIntervalEnd;
            co::Location::Ptr prevLocation;
            LocationTimepointTuple::Ptr startTuple, endTuple;

            //const std::vector<symbols::constants::Location::Ptr>& locations = roleTimeline.getLocations();
            //const std::vector<solvers::temporal::Interval>& getIntervals = roleTimeline.getIntervals();
            const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
            std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
            for(; fit != ftrs.end(); ++fit)
            {
                symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
                solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

                std::cout << "Location: " << location->toString() << " -- interval: " << interval.toString() << std::endl;

                endTuple = tupleMap[ LocationTimePointPair(location, interval.getFrom()) ];

                // Find start node: Tuple of location  and interval.getFrom()
                if(prevIntervalEnd)
                {
                    startTuple = tupleMap[ LocationTimePointPair(prevLocation, prevIntervalEnd) ];
                    std::vector< WeightedEdge::Ptr > edges = spaceTimeGraph->getEdges<WeightedEdge>(startTuple, endTuple);
                    if(edges.empty())
                    {
                        WeightedEdge::Ptr weightedEdge(new WeightedEdge(startTuple, endTuple, capacity));
                        spaceTimeGraph->addEdge(weightedEdge);
                    } else {
                        if(edges.size() > 1)
                        {
                            throw std::runtime_error("MissionPlanner: multiple capacity edges detected");
                        }

                        WeightedEdge::Ptr& existingEdge = edges[0];
                        double existingCapacity = existingEdge->getWeight();
                        if(existingCapacity < std::numeric_limits<WeightedEdge::value_t>::max())
                        {
                            capacity += existingCapacity;
                            existingEdge->setWeight(capacity, 0);
                        }
                    }
                }

                prevIntervalEnd = interval.getTo();
                prevLocation = location;
            }
        }

        {
            std::string filename = "/tmp/space-time-graph-with-timeline.dot";
            graph_analysis::io::GraphIO::write(filename, spaceTimeGraph);
            std::cout << "Written temporally expanded graph to: " << filename << std::endl;
            std::cout << "(e.g. view with 'xdot " << filename << "'" << ")" << std::endl;
        }
    }


    // Analyse the cost of the planning approach



//    MissionPlanner missionPlanner(mission);
//    CandidateMissions missions = missionPlanner.solve();

    return 0;
}
