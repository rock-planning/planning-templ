#include <templ/io/MissionReader.hpp>
//#include <templ/MissionPlanner.hpp>

#include <templ/solvers/csp/RoleDistribution.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>

#include <templ/LocationTimepointTuple.hpp>
#include <graph_analysis/WeightedEdge.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/algorithms/MultiCommodityMinCostFlow.hpp>
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
    using namespace graph_analysis;
    namespace pa = templ::solvers::temporal::point_algebra;
    namespace co = templ::symbols::constants;

    BaseGraph::Ptr spaceTimeGraph = BaseGraph::getInstance();
    // Allow to find an existing tuple
    typedef std::pair< co::Location::Ptr, pa::TimePoint::Ptr> LocationTimePointPair;
    std::map< LocationTimePointPair, LocationTimepointTuple::Ptr > tupleMap;

    uint32_t commodities = 0;
    {

        std::vector<pa::TimePoint::Ptr> timepoints = mission.getTimepoints();
        pa::TimePointComparator tpc(mission.getTemporalConstraintNetwork());
        std::sort(timepoints.begin(), timepoints.end(), [&tpc](const pa::TimePoint::Ptr& a, const pa::TimePoint::Ptr& b)
                {
                    if(a == b)
                    {
                        return false;
                    }

                    if( tpc.lessThan(a,b) )
                    {
                        std::cout << a->toString() << " is less than " << b->toString() << std::endl;
                        return true;
                    }
                    return false;
                }
        );

        std::vector<pa::TimePoint::Ptr>::const_iterator sit = timepoints.begin();
        for(; sit != timepoints.end(); ++sit)
        {
            std::cout << "Timepoint: " << (*sit)->toString() << std::endl;
        }

        std::vector<co::Location::Ptr> locations = mission.getLocations();


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
            if( mission.getTemporalConstraintNetwork()->isConsistent())
            {
                std::cout << "Network is consistent" << std::endl;
            }
            std::string filename = "/tmp/mission-temporally-constrained-network.dot";
            graph_analysis::io::GraphIO::write(filename, mission.getTemporalConstraintNetwork()->getGraph());
            std::cout << "Written temporal constraint network to: " << filename << std::endl;
            std::cout << "(e.g. view with 'xdot " << filename << "'" << ")" << std::endl;
        }

        // Per Role --> add capacities (in terms of capability of carrying a
        // payload)
        std::map<Role, RoleTimeline>::const_iterator rit = timelines.begin();
        for(; rit != timelines.end(); ++rit)
        {
            // infer connections from timeline
            // sequentially ordered timeline
            // locations and timeline
            // connection from (l0, i0_end) ---> (l1, i1_start)
            //
            const Role& role = rit->first;
            RoleTimeline roleTimeline = rit->second;
            roleTimeline.sortByTime();

            // Check if this item is a payload -- WARNING: this is domain specific
            owlapi::model::OWLOntologyAsk ask(organizationModel->ontology());
            owlapi::model::IRI payloadClass = owlapi::vocabulary::OM::resolve("Payload");
            if(payloadClass == role.getModel() || ask.isSubClassOf(role.getModel(), owlapi::vocabulary::OM::resolve("Payload")))
            {
                std::cout << "Delay handling of payload" << std::endl;
                ++commodities;
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

                // create tuple if it does not exist, otherwise reuse
                endTuple = tupleMap[ LocationTimePointPair(location, interval.getFrom()) ];
                endTuple->addRole(role);

                // Find start node: Tuple of location  and interval.getFrom()
                if(prevIntervalEnd)
                {
                    startTuple = tupleMap[ LocationTimePointPair(prevLocation, prevIntervalEnd) ];
                    startTuple->addRole(role);

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

    }

    // Compute multicommodity min-cost flow problem for payloads
    {
        using namespace graph_analysis::algorithms;

        BaseGraph::Ptr flowGraph = BaseGraph::getInstance();
        // uint32_t commodities --> see above: counted from existing payload
        // roles
        uint32_t edgeCapacityUpperBound = 60;

        std::map<Vertex::Ptr, Vertex::Ptr> commodityToSpace;
        std::map<Vertex::Ptr, Vertex::Ptr> spaceToCommodity;

        VertexIterator::Ptr vertexIt = spaceTimeGraph->getVertexIterator();
        while(vertexIt->next())
        {
            LocationTimepointTuple::Ptr tuple = boost::dynamic_pointer_cast<LocationTimepointTuple>(vertexIt->current());

            MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex(new MultiCommodityMinCostFlow::vertex_t(commodities));
            commodityToSpace[multicommodityVertex] = tuple;
            spaceToCommodity[tuple] = multicommodityVertex;

            flowGraph->addVertex(multicommodityVertex);
        }

        {
            EdgeIterator::Ptr edgeIt = spaceTimeGraph->getEdgeIterator();
            while(edgeIt->next())
            {
                WeightedEdge::Ptr edge = boost::dynamic_pointer_cast<WeightedEdge>(edgeIt->current());

                Vertex::Ptr source = edge->getSourceVertex();
                Vertex::Ptr target = edge->getTargetVertex();

                MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge(new MultiCommodityMinCostFlow::edge_t(commodities));
                multicommodityEdge->setSourceVertex( spaceToCommodity[source] );
                multicommodityEdge->setTargetVertex( spaceToCommodity[target] );

                double weight = edge->getWeight();
                uint32_t bound = 0;
                if(weight == std::numeric_limits<double>::max())
                {
                    bound = std::numeric_limits<uint32_t>::max();
                } else {
                    bound = static_cast<uint32_t>(weight);
                }

                multicommodityEdge->setCapacityUpperBound(bound);
                for(size_t i = 0; i < commodities; ++i)
                {
                    multicommodityEdge->setCommodityCapacityUpperBound(i, bound);
                }

                flowGraph->addEdge(multicommodityEdge);
            }
        }


        std::vector<Role> commodityRoles;

        std::map<Role, RoleTimeline>::const_iterator rit = timelines.begin();
        for(; rit != timelines.end(); ++rit)
        {
            const Role& role = rit->first;
            const RoleTimeline& roleTimeline = rit->second;

            // Check if this item is a payload -- WARNING: this is domain specific
            owlapi::model::OWLOntologyAsk ask(organizationModel->ontology());
            owlapi::model::IRI payloadClass = owlapi::vocabulary::OM::resolve("Payload");
            if(payloadClass == role.getModel() || ask.isSubClassOf(role.getModel(), owlapi::vocabulary::OM::resolve("Payload")))
            {
                std::cout << "Delay handling of payload" << std::endl;
            } else {
                // no payload -- no need
                continue;
            }

            commodityRoles.push_back(role);
            size_t commodityId = commodityRoles.size() - 1;

            const std::vector<FluentTimeResource>& ftrs = roleTimeline.getFluentTimeResources();
            std::vector<FluentTimeResource>::const_iterator fit = ftrs.begin();
            for(; fit != ftrs.end(); ++fit)
            {
                symbols::constants::Location::Ptr location = roleTimeline.getLocation(*fit);
                solvers::temporal::Interval interval = roleTimeline.getInterval(*fit);

                // Get the tuple in the graph
                LocationTimepointTuple::Ptr currentTuple = tupleMap[ LocationTimePointPair(location, interval.getFrom()) ];
                currentTuple->addRole(role);
                Vertex::Ptr vertex = spaceToCommodity[currentTuple];
                assert(vertex);
                MultiCommodityMinCostFlow::vertex_t::Ptr multicommodityVertex =
                    boost::dynamic_pointer_cast<MultiCommodityMinCostFlow::vertex_t>(vertex);

                if(fit == ftrs.begin())
                {
                    multicommodityVertex->setCommoditySupply(commodityId, 1);
                } else {
                    multicommodityVertex->setCommoditySupply(commodityId, -1);
                }
            }

            // First entry can be interpreted as source
            // following entries as demands
        }

        MultiCommodityMinCostFlow minCostFlow(flowGraph, commodities);
        uint32_t cost = minCostFlow.run();
        std::cout << "Ran flow optimization: " << cost << std::cout;
        minCostFlow.storeResult();

        std::cout << "Update after flow optimization" << std::cout;
        {
            // Update commodites after flow optimization
            EdgeIterator::Ptr edgeIt = flowGraph->getEdgeIterator();
            while(edgeIt->next())
            {
                MultiCommodityMinCostFlow::edge_t::Ptr multicommodityEdge =
                    boost::dynamic_pointer_cast<MultiCommodityMinCostFlow::edge_t>(edgeIt->current());

                for(int i = 0; i < commodityRoles.size(); ++i)
                {
                    uint32_t flow = multicommodityEdge->getCommodityFlow(i);
                    if(flow > 0)
                    {

                        const Role& role = commodityRoles[i];
                        Vertex::Ptr sourceLocation = commodityToSpace[multicommodityEdge->getSourceVertex()];
                        Vertex::Ptr targetLocation = commodityToSpace[multicommodityEdge->getTargetVertex()];
                        assert(sourceLocation && targetLocation);

                        boost::dynamic_pointer_cast<LocationTimepointTuple>(sourceLocation)->addRole(role);
                        boost::dynamic_pointer_cast<LocationTimepointTuple>(targetLocation)->addRole(role);
                    }
                }
            }
        }


        {
            std::string filename = "/tmp/mission-flow-optimization.dot";
            graph_analysis::io::GraphIO::write(filename, flowGraph);
            std::cout << "Written flow network: " << filename << std::endl;
            std::cout << "(e.g. view with 'xdot " << filename << "'" << ")" << std::endl;
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
