#include "SolutionAnalysis.hpp"
#include "../RoleInfoVertex.hpp"
#include "../utils/PathConstructor.hpp"

#include <organization_model/Algebra.hpp>
#include <graph_analysis/algorithms/DFS.hpp>
#include <boost/bind.hpp>

namespace templ {
namespace solvers {

SolutionAnalysis::SolutionAnalysis(const Mission::Ptr& mission, const SpaceTime::Network& solution)
    : mpMission(mission)
    , mSolutionNetwork(solution)
    , mTimepointComparator(mission->getTemporalConstraintNetwork())
{
    mResourceRequirements = Mission::getResourceRequirements(mpMission);
}

std::set<Role> SolutionAnalysis::getRequiredRoles(size_t minRequirement) const
{
    using namespace graph_analysis;

    std::set<Role> requiredRoles;
    std::map<Role, size_t> mRoleUsage;

    assert(mSolutionNetwork.getGraph());

    VertexIterator::Ptr vertexIt = mSolutionNetwork.getGraph()->getVertexIterator();
    while(vertexIt->next())
    {
        SpaceTime::Network::tuple_t::Ptr currentTuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(vertexIt->current());
        assert(currentTuple);
        std::set<Role> involvedRoles = currentTuple->getAllRoles();
        for(const Role& role : involvedRoles)
        {
            mRoleUsage[role] += 1;
        }
    }

    for(std::pair<Role, size_t> pair : mRoleUsage)
    {
        if(pair.second >= minRequirement)
        {
            requiredRoles.insert(pair.first);
        }
    }
    return requiredRoles;
}



organization_model::ModelPool SolutionAnalysis::getMinAvailableResources(const csp::FluentTimeResource& ftr) const
{
    symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(ftr.getFluent());
    assert(location);

    std::vector<organization_model::ModelPool> availableResources = getAvailableResources(location, ftr.getInterval());

    using namespace organization_model;
    // return the minimum available resources of the
    ModelPool minAvailableResources = organization_model::Algebra::min( availableResources );

    // Infer functionality from this set of resources
    OrganizationModelAsk ask(mpMission->getOrganizationModel(),
            minAvailableResources,
            true);
    // Creating model pool from available functionalities
    ModelPool functionalities = ask.getSupportedFunctionalities();
    return organization_model::Algebra::max(minAvailableResources, functionalities);
}

organization_model::ModelPool SolutionAnalysis::getMaxAvailableResources(const csp::FluentTimeResource& ftr) const
{
    symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(ftr.getFluent());
    assert(location);

    std::vector<organization_model::ModelPool> availableResources = getAvailableResources(location, ftr.getInterval());

    using namespace organization_model;
    // return the minimum available resources of the
    ModelPool maxAvailableResources = Algebra::max( availableResources );

    // Infer functionality from this set of resources
    OrganizationModelAsk ask(mpMission->getOrganizationModel(),
            maxAvailableResources,
            true);
    // Creating model pool from available functionalities
    ModelPool functionalities = ask.getSupportedFunctionalities();
    return organization_model::Algebra::max(maxAvailableResources, functionalities);
}

std::vector<organization_model::ModelPool> SolutionAnalysis::getAvailableResources(const symbols::constants::Location::Ptr& location, const solvers::temporal::Interval& interval) const
{
    using namespace temporal::point_algebra;

    std::vector<organization_model::ModelPool> modelPools;

    // Iterate over all known timepoints and check if the timepoint belongs to
    // the interval (the list of timepoints is sorted)
    TimePoint::PtrList timepoints = mSolutionNetwork.getTimepoints();

    assert(!timepoints.empty());

    for(TimePoint::Ptr timepoint : timepoints)
    {
        if( mTimepointComparator.inInterval(timepoint, interval.getFrom(), interval.getTo()) )
        {
            // identified relevant tuple
            SpaceTime::Network::tuple_t::Ptr tuple = mSolutionNetwork.tupleByKeys(location, timepoint);
            Role::Set foundRoles = tuple->getRoles("assigned");
            Role::List roles(foundRoles.begin(), foundRoles.end());
            organization_model::ModelPool currentPool = Role::getModelPool(roles);

            modelPools.push_back(currentPool);
        }
    }
    return modelPools;
}

SolutionAnalysis::MinMaxModelPools SolutionAnalysis::getRequiredResources(const symbols::constants::Location::Ptr& location, const solvers::temporal::Interval& interval) const
{
    using namespace temporal::point_algebra;
    MinMaxModelPools minMaxModelPools;

    std::vector<solvers::csp::FluentTimeResource>::const_iterator cit = mResourceRequirements.begin();
    for(; cit != mResourceRequirements.end(); ++cit)
    {
        const solvers::csp::FluentTimeResource& ftr = *cit;

        symbols::constants::Location::Ptr ftrLocation = ftr.getLocation();
        if(location != ftrLocation)
        {
            continue;
        }

        if( mTimepointComparator.hasIntervalOverlap(ftr.getInterval().getFrom(),
                    ftr.getInterval().getTo(),
                    interval.getFrom(),
                    interval.getTo()))
        {
            minMaxModelPools.first.push_back( ftr.minCardinalities );
            minMaxModelPools.second.push_back( ftr.maxCardinalities );
        }
    }

    return minMaxModelPools;
}

SolutionAnalysis::MinMaxModelPools SolutionAnalysis::getRequiredResources(const csp::FluentTimeResource& ftr) const
{
    using namespace temporal::point_algebra;
    MinMaxModelPools minMaxModelPools;

    std::vector<solvers::csp::FluentTimeResource>::const_iterator cit = mResourceRequirements.begin();
    for(; cit != mResourceRequirements.end(); ++cit)
    {
        const solvers::csp::FluentTimeResource& requirementFtr = *cit;
        if(ftr.getLocation() == requirementFtr.getLocation() &&
                ftr.getInterval() == requirementFtr.getInterval())
        {
            minMaxModelPools.first.push_back(ftr.minCardinalities);
            minMaxModelPools.second.push_back(ftr.maxCardinalities);
            return minMaxModelPools;
        }
    }

    throw std::invalid_argument("templ::solvers::SolutionAnalysis::getRequiredResources: could not find the corrensponding requirement for an existing fluent time resource");
}

void SolutionAnalysis::analyse()
{
//    // a collect all requirements of the mission -- as translated from the
//    // persistence conditions
//    std::vector<solvers::csp::FluentTimeResource> requirements = Mission::getResourceRequirements(mpMission);
//    for(const solvers::csp::FluentTimeResource& ftr : requirements)
//    {
//        analyse(ftr);
//    }
}

void SolutionAnalysis::analyse(const solvers::csp::FluentTimeResource& ftr)
{
    double d = degreeOfFulfillment(ftr);
    // Main spatio temporal requirement
    // needs to be checked upon fulfillment
    //
    // -- partial fulfillment or full fulfillment --
    //std::vector<solvers::csp::FluentTimeResource& ftr = solution.collectRelated(ftr);
}

double SolutionAnalysis::degreeOfFulfillment(const solvers::csp::FluentTimeResource& ftr)
{
//    std::set<organization_model::Functionality> requiredFunctionalities = ftr.getFunctionalities();
//
//    std::set<organization_model::Functionality> availableFunctionalities = solution.getAvailableFunctionalities(dynamic_pointer_cast<symbols::constants::Location>(ftr.getFluent()), interval);
    return 0.0;
}

organization_model::ModelPool SolutionAnalysis::getMinResourceRequirements(const csp::FluentTimeResource& ftr) const
{
    using namespace organization_model;
    return getRequiredResources(ftr).first.front();
}

organization_model::ModelPool SolutionAnalysis::getMaxResourceRequirements(const csp::FluentTimeResource& ftr) const
{
    using namespace organization_model;
    return getRequiredResources(ftr).second.front();
}

organization_model::ModelPoolDelta SolutionAnalysis::getMinMissingResourceRequirements(const solvers::csp::FluentTimeResource& ftr) const
{
    using namespace organization_model;
    ModelPool requiredResources = getMinResourceRequirements(ftr);
    ModelPool maxAvailableResources = getMaxAvailableResources(ftr);

    // Infer functionality from this set of resources
    OrganizationModelAsk ask(mpMission->getOrganizationModel(),
            maxAvailableResources,
            true);
    // Creating model pool from available functionalities
    ModelPool functionalities = ask.getSupportedFunctionalities();
    ModelPool availableResources = organization_model::Algebra::max(maxAvailableResources, functionalities);

    return Algebra::delta(requiredResources, availableResources);
}

organization_model::ModelPoolDelta SolutionAnalysis::getMaxMissingResources(const solvers::csp::FluentTimeResource& ftr) const
{
    using namespace organization_model;
    ModelPool requiredResources = getMinResourceRequirements(ftr);
    ModelPool minAvailableResources = getMinAvailableResources(ftr);

    // Infer functionality from this set of resources
    OrganizationModelAsk ask(mpMission->getOrganizationModel(),
            minAvailableResources,
            true);
    // Creating model pool from available functionalities
    ModelPool functionalities = ask.getSupportedFunctionalities();
    ModelPool availableResources = organization_model::Algebra::min(minAvailableResources, functionalities);

    return Algebra::delta(requiredResources, availableResources);
}

graph_analysis::BaseGraph::Ptr SolutionAnalysis::toHyperGraph()
{
    using namespace graph_analysis;
    BaseGraph::Ptr hyperGraph = mSolutionNetwork.getGraph()->copy();

    size_t minUsage = 2;
    std::set<Role> roles = getRequiredRoles(minUsage);

    // Create set of vertices that represents
    // each role
    std::map<Role, RoleInfoVertex::Ptr> role2VertexMap;
    for(const Role& role : roles)
    {
        RoleInfoVertex::Ptr roleInfo(new RoleInfoVertex());
        roleInfo->addRole(role);
        role2VertexMap[role] = roleInfo;
        hyperGraph->addVertex(roleInfo);
    }


    // Iterate over the set of vertices in the solution and
    // create an edge to each RoleInfoVertex for a required role
    VertexIterator::Ptr vertexIt = mSolutionNetwork.getGraph()->getVertexIterator();
    while(vertexIt->next())
    {
        SpaceTime::Network::tuple_t::Ptr roleInfo = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(vertexIt->current());
        std::set<Role> roles = roleInfo->getRoles("assigned");
        if(roles.empty())
        {
            continue;
        }
        for(const Role& role : roles)
        {
            Edge::Ptr edge(new Edge("requires"));
            edge->setSourceVertex(roleInfo);

            RoleInfoVertex::Ptr targetRoleVertex = role2VertexMap[role];
            edge->setTargetVertex(targetRoleVertex);
            hyperGraph->addEdge(edge);
        }
    }

    // Iterate over the set of edges in the solution and
    // create and edge to each RoleInfoVertex for a required role
    EdgeIterator::Ptr edgeIt = mSolutionNetwork.getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {
        SpaceTime::Network::edge_t::Ptr roleInfo = dynamic_pointer_cast<SpaceTime::Network::edge_t>(edgeIt->current());
        std::set<Role> roles = roleInfo->getRoles("assigned");
        if(roles.empty())
        {
            continue;
        }

        Vertex::PtrList vertices;
        vertices.push_back(roleInfo->getSourceVertex());
        vertices.push_back(roleInfo->getTargetVertex());

        std::stringstream edgeLabel;
        edgeLabel << "vertices: [";
        edgeLabel << hyperGraph->getVertexId( roleInfo->getSourceVertex() );
        edgeLabel << ", ";
        edgeLabel << hyperGraph->getVertexId( roleInfo->getTargetVertex() );
        edgeLabel << "]";

        HyperEdge::Ptr hyperEdge(new HyperEdge(vertices, edgeLabel.str()));
        hyperGraph->addHyperEdge(hyperEdge);

        for(const Role& role : roles)
        {
            Edge::Ptr edge(new Edge("requires"));
            edge->setSourceVertex(hyperEdge);
            RoleInfoVertex::Ptr targetRoleVertex = role2VertexMap[role];
            edge->setTargetVertex(targetRoleVertex);
            hyperGraph->addEdge(edge);
        }
    }
    return hyperGraph;
}

Plan SolutionAnalysis::computePlan() const
{
    Plan plan(mpMission);


    using namespace solvers::temporal;
    using namespace symbols::constants;
    point_algebra::TimePoint::PtrList timepoints = mpMission->getTimepoints();    Location::PtrList locations = mpMission->getLocations();

    assert(!timepoints.empty());
    point_algebra::TimePoint::Ptr startingTimepoint = timepoints.front();

    std::set<Role> requiredRoles = getRequiredRoles(2);
    for(const Role& role : requiredRoles)
    {
        SpaceTime::Network::tuple_t::Ptr startTuple;
        // Find the start point of a role
        for(const Location::Ptr& location : locations)
        {
            try {
                SpaceTime::Network::tuple_t::Ptr tuple = mSolutionNetwork.tupleByKeys(location, startingTimepoint);

                Role::Set assignedRoles = tuple->getRoles("assigned");
                if( assignedRoles.find(role) != assignedRoles.end())
                {
                    startTuple = tuple;
                    break;
                }
            } catch(const std::exception& e)
            {
                LOG_WARN_S << e.what() << " " << location->toString() << " " << startingTimepoint->toString();
            }
        }

        if(!startTuple)
        {
            LOG_WARN_S << "Could not find start tuple for role " + role.toString() << " solution seems to be incomplete";
            continue;
        }


        // Finding the starting tuple
        using namespace graph_analysis::algorithms;
        // use SpaceTime::Network, which contains information on role for each edge
        // after update from the flow graph
        // foreach role -- find starting point and follow path
        PathConstructor::Ptr pathConstructor(new PathConstructor(role, "assigned"));
        Skipper skipper = boost::bind(&PathConstructor::isInvalidTransition, pathConstructor,_1);
        DFS dfs(mSolutionNetwork.getGraph(), pathConstructor, skipper);
        dfs.run(startTuple);

        std::vector<graph_analysis::Vertex::Ptr> path = pathConstructor->getPath();
        path.insert(path.begin(), startTuple);
        plan.add(role,path);
    }

    return plan;
}

std::string SolutionAnalysis::toString(size_t indent) const
{
    std::string hspace(indent, ' ');
    std::stringstream ss;
    ss << hspace << "SolutionAnalysis:" << std::endl;
    ss << hspace << "    required roles: " << Role::toString( getRequiredRoles() ) << std::endl;

    for(const solvers::csp::FluentTimeResource& ftr : mResourceRequirements)
    {
        ss << ftr.toString(indent + 4) << std::endl;
        ss << getMinAvailableResources(ftr).toString(indent + 4) << std::endl;
    }

    Plan plan = computePlan();
    ss << hspace << "Resulting plan:" << std::endl;
    ss << plan.toString(indent + 4);

    return ss.str();
}

} // end namespace solvers
} // end namespace templ
