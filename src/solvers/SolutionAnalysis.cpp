#include "SolutionAnalysis.hpp"
#include "csp/FluentTimeResource.hpp"
#include "../RoleInfoVertex.hpp"

namespace templ {
namespace solvers {

SolutionAnalysis::SolutionAnalysis(const Mission::Ptr& mission, const SpaceTime::Network& solution)
    : mpMission(mission)
    , mSolutionNetwork(solution)
{
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

std::vector<organization_model::ModelPool> SolutionAnalysis::getAvailableResources(const symbols::constants::Location::Ptr& location, const solvers::temporal::Interval& interval) const
{
    using namespace temporal::point_algebra;

    std::vector<organization_model::ModelPool> modelPools;

    TimePoint::PtrList timepoints = mSolutionNetwork.getTimepoints();
    bool partOfInterval = false;
    for(TimePoint::Ptr timepoint : timepoints)
    {
        if(interval.getFrom() == timepoint)
        {
            partOfInterval = true;
        }

        if(partOfInterval)
        {
            // identified relevant tuple
            SpaceTime::Network::tuple_t::Ptr tuple = mSolutionNetwork.tupleByKeys(location, timepoint);
            Role::Set foundRoles = tuple->getRoles("assigned");
            Role::List roles(foundRoles.begin(), foundRoles.end());
            organization_model::ModelPool currentPool = Role::getModelPool(roles);

            graph_analysis::EdgeIterator::Ptr inEdgeIt = mSolutionNetwork.getGraph()->getInEdgeIterator(tuple);
            while(inEdgeIt->next())
            {
                RoleInfoWeightedEdge::Ptr roleInfoEdge = dynamic_pointer_cast<RoleInfoWeightedEdge>(inEdgeIt->current());
                assert(roleInfoEdge);

                Role::Set mobileRoles = roleInfoEdge->getRoles("assigned");
                for(const Role& mobileRole : mobileRoles)
                {
                    currentPool[mobileRole.getModel()] += 1;
                }
            }

            modelPools.push_back(currentPool);
        }

        if(interval.getTo() == timepoint)
        {
            partOfInterval = false;
        }
    }
    return modelPools;
}
//

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

graph_analysis::BaseGraph::Ptr SolutionAnalysis::toHyperGraph()
{
    using namespace graph_analysis;
    BaseGraph::Ptr hyperGraph = mSolutionNetwork.getGraph()->copy();

    std::set<Role> roles = getRequiredRoles(2);
    std::map<Role, RoleInfoVertex::Ptr> role2VertexMap;
    for(const Role& role : roles)
    {
        RoleInfoVertex::Ptr roleInfo(new RoleInfoVertex());
        roleInfo->addRole(role);
        role2VertexMap[role] = roleInfo;
        hyperGraph->addVertex(roleInfo);
    }


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

} // end namespace solvers
} // end namespace templ
