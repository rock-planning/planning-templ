#include "Solution.hpp"
#include <limits>
#include <functional>
#include <graph_analysis/algorithms/DFS.hpp>
#include <moreorg/Agent.hpp>
#include <moreorg/facades/Robot.hpp>

#include "../utils/PathConstructor.hpp"
#include "../Mission.hpp"
#include "../constraints/ModelConstraint.hpp"
#include "../utils/PathConstructor.hpp"

using namespace graph_analysis;

namespace templ {
namespace solvers {

Solution::Solution()
    : mSpaceTimeNetwork()
{}

Solution::Solution(const SpaceTime::Network& network, const moreorg::OrganizationModel::Ptr& om)
    : mSpaceTimeNetwork(network)
    , mOrganizationModelAsk(om)
{}


/**
 *  Get all Agent Roles which are involved in this solution
 */
Role::Set Solution::getAgentRoles() const
{
    std::set<Role> allRoles;
    temporal::point_algebra::TimePoint::Ptr startTp = *getTimepoints().begin();
    for(const symbols::constants::Location::Ptr& location : getLocations())
    {
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr roleInfoTuple =
            mSpaceTimeNetwork.tupleByKeys(location, startTp);
        std::set<Role> roles = roleInfoTuple->getRoles(RoleInfo::ASSIGNED);
        allRoles.insert(roles.begin(), roles.end());
    }
    return allRoles;
}

SpaceTime::RoleInfoSpaceTimeTuple::Ptr Solution::getStart(const Role& role) const
{
    temporal::point_algebra::TimePoint::Ptr startTp = *getTimepoints().begin();
    for(const symbols::constants::Location::Ptr& location : getLocations())
    {
        const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& roleInfoTuple =
            mSpaceTimeNetwork.tupleByKeys(location, startTp);
        if(roleInfoTuple->hasRole(role, RoleInfo::ASSIGNED))
        {
            return roleInfoTuple;
        }

    }
    throw std::runtime_error("templ::Solution::getStart: failed to find starting position for"
            " role '" + role.toString() + "'");
}

/**
 * Get the route for this particular role
 * \return List of SpaceTime::RoleInfoSpaceTimeTuple::Ptr
 */
SpaceTime::Route Solution::getRoute(const Role& role) const
{
    SpaceTime::RoleInfoSpaceTimeTuple::Ptr startTuple = getStart(role);

    LOG_WARN_S << "Route: " << role.toString() << ": start at " << startTuple->toString();

    using namespace graph_analysis::algorithms;
    // use SpaceTime::Network, which contains information on role for each edge
    // after update from the flow graph
    // foreach role -- find starting point and follow path
    PathConstructor::Ptr pathConstructor =
        make_shared<PathConstructor>(role,
                RoleInfo::TagTxt[ RoleInfo::ASSIGNED ]);
    Skipper skipper = std::bind(&PathConstructor::isInvalidTransition,
            pathConstructor,std::placeholder::_1);
    DFS dfs(mSpaceTimeNetwork.getGraph(), pathConstructor, skipper);
    dfs.run(startTuple);

    std::vector<graph_analysis::Vertex::Ptr> path = pathConstructor->getPath();
    LOG_WARN_S << "Route: " << role.toString() << ": " << path.size();
    SpaceTime::Route route;
    for(const graph_analysis::Vertex::Ptr& v : path)
    {
        route.push_back( dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(v) );
    }
    return route;
}

/**
 * Allow to change a solution
 * \return Location where the role has been removed
 */
SpaceTime::RoleInfoSpaceTimeTuple::Ptr Solution::removeRole(const Role& role,
        const temporal::point_algebra::TimePoint::Ptr& timepoint,
        const symbols::constants::Location::Ptr& location,
        RoleInfo::Tag tag)
{
    if(location)
    {
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr roleInfoTuple = mSpaceTimeNetwork.tupleByKeys(location, timepoint);
        roleInfoTuple->removeRole(role, tag);

        // 1. check that the role is also remove from the incoming and outgoing
        // edges
        Edge::PtrList inEdges = mSpaceTimeNetwork.getGraph()->getInEdges(roleInfoTuple);
        for(const Edge::Ptr& inEdge : inEdges)
        {
            RoleInfoWeightedEdge::Ptr e =
                dynamic_pointer_cast<RoleInfoWeightedEdge>(inEdge);

            if(e->hasRole(role, tag))
            {
                e->removeRole(role, tag);

                if(e->getRoles(RoleInfo::ASSIGNED).empty())
                {
                    if( e->getWeight() != std::numeric_limits<double>::max() )
                    {
                        mSpaceTimeNetwork.getGraph()->removeEdge(e);
                    }
                }
                break;
            }
        }

        Edge::PtrList outEdges = mSpaceTimeNetwork.getGraph()->getOutEdges(roleInfoTuple);
        for(const Edge::Ptr& outEdge : outEdges)
        {
            RoleInfoWeightedEdge::Ptr e =
                dynamic_pointer_cast<RoleInfoWeightedEdge>(outEdge);

            if(e->hasRole(role, tag))
            {
                e->removeRole(role, tag);

                if(e->getAllRoles().empty())
                {
                    if( e->getWeight() != std::numeric_limits<double>::max() )
                    {
                        mSpaceTimeNetwork.getGraph()->removeEdge(e);
                    }
                }
                break;
            }
        }

        return roleInfoTuple;
    } else {
        for(const symbols::constants::Location::Ptr& l : getLocations())
        {
            SpaceTime::RoleInfoSpaceTimeTuple::Ptr roleInfoTuple = mSpaceTimeNetwork.tupleByKeys(l, timepoint);
            if(roleInfoTuple->hasRole(role, tag))
            {
                roleInfoTuple->removeRole(role, tag);
                return roleInfoTuple;
            }
        }
    }
    throw std::runtime_error("templ::Solution::deleteRole failed to find role"
            " '" + role.toString() + "' for timepoint '" + timepoint->toString() + "'");
}

/**
 * Add a role to to a space time tuple
 * \return the updated role info tuple
 */
const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& Solution::addRole(const Role& role,
        const temporal::point_algebra::TimePoint::Ptr& timepoint,
        const symbols::constants::Location::Ptr& location,
        RoleInfo::Tag tag)
{
    const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& roleInfoTuple = mSpaceTimeNetwork.tupleByKeys(location, timepoint);
    if(roleInfoTuple)
    {
        roleInfoTuple->addRole(role, tag);
        return roleInfoTuple;
    }
    throw std::runtime_error("templ::solvers::Solution::addRole failed to find"
            " role '" + role.toString() + "' at '" + location->toString() + "/"
            + timepoint->toString() + "'");
}

SpaceTime::RoleInfoSpaceTimeTuple::Ptr Solution::addRole(const Role& role,
        const temporal::point_algebra::TimePoint::Ptr& from,
        const temporal::point_algebra::TimePoint::Ptr& to,
        const symbols::constants::Location::Ptr& location,
        RoleInfo::Tag tag)
{
    bool inRoute = false;
    SpaceTime::RoleInfoSpaceTimeTuple::Ptr prevTuple;
    size_t startIndex = 0;

    // check if there is already a previous spacetimepoint, where this role
    // is associated with

    symbols::constants::Location::PtrList locations = mSpaceTimeNetwork.getValues();

    temporal::point_algebra::TimePoint::PtrList timepoints = getTimepoints();
    for(size_t t = 0; t < timepoints.size(); ++t)
    {
        if(timepoints[t] == from)
        {
            startIndex = t;
            inRoute = true;

            // where is the origin of the role (if this is not the starting
            // location)
            if(t > 0)
            {
                for(const symbols::constants::Location::Ptr& l : locations)
                {
                    const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& roleInfoTuple
                        = mSpaceTimeNetwork.tupleByKeys(l, timepoints[t-1]);
                    if(roleInfoTuple->hasRole(role, tag))
                    {
                        // found the location
                        prevTuple = roleInfoTuple;
                        break;
                    }
                }
            }
        } else if(!inRoute)
        {
            continue;
        }

        addRole(role, timepoints[t], location, tag);

        const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& roleInfoTuple = mSpaceTimeNetwork.tupleByKeys(location, timepoints[t]);
        if(prevTuple)
        {
            std::vector<RoleInfoWeightedEdge::Ptr> edges = mSpaceTimeNetwork.getGraph()->getEdges<RoleInfoWeightedEdge>(prevTuple, roleInfoTuple);
            RoleInfoWeightedEdge::Ptr edge;
            if(edges.empty())
            {
                edge = make_shared<RoleInfoWeightedEdge>(prevTuple, roleInfoTuple, 0);
                mSpaceTimeNetwork.getGraph()->addEdge(edge);
            } else if(edges.size() == 1)
            {
                edge = edges[0];
            } else {
                throw std::runtime_error("templ::solvers::Solution: multiple"
                        "edges encountered");
            }

            edge->addRole(role, tag);

            if(prevTuple->first() == roleInfoTuple->first())
            {
                edge->setWeight(std::numeric_limits<double>::max());
            } else
            {
                Role::Set allRoles = edge->getRoles({ RoleInfo::ASSIGNED });
                uint32_t totalTransportCapacity = 0;
                for(const Role& r : allRoles)
                {
                    moreorg::facades::Robot robot(r.getModel(), mOrganizationModelAsk);
                    uint32_t tcap = robot.getTransportCapacity();
                    totalTransportCapacity += tcap;
                }
                edge->setWeight(totalTransportCapacity);
            }
        }
        prevTuple = roleInfoTuple;

        if(timepoints[t] == to)
        {
            inRoute = false;

            // where is the target of the role (if this is not the final location)
            if(t < timepoints.size() - 1)
            {
                for(const symbols::constants::Location::Ptr& l : locations)
                {
                    const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& targetTuple
                        = mSpaceTimeNetwork.tupleByKeys(l, timepoints[t+1]);
                    if(roleInfoTuple->hasRole(role, tag))
                    {
                        std::vector<RoleInfoWeightedEdge::Ptr> outEdges = mSpaceTimeNetwork.getGraph()->getEdges<RoleInfoWeightedEdge>(prevTuple, targetTuple);

                        if(outEdges.size() == 1)
                        {
                            outEdges[0]->addRole(role, tag);
                        } else if(outEdges.empty())
                        {
                            // create edge
                            RoleInfoWeightedEdge::Ptr edge = make_shared<RoleInfoWeightedEdge>(prevTuple, roleInfoTuple, 0);
                            mSpaceTimeNetwork.getGraph()->addEdge(edge);
                            edge->addRole(role, tag);
                        } else {
                            throw
                                std::runtime_error("templ::solvers::Solution::addRole:"
                                        "multiple edges encountered between two"
                                        "SpaceTimePoints");
                        }
                        break;
                    }
                }
            }
            break;
        }
    }
    return mSpaceTimeNetwork.tupleByKeys(location, timepoints[startIndex]);
}


/**
 * Retrieve the edge between two RoleIntoTuple::Ptr
 */
RoleInfoWeightedEdge::PtrList Solution::getEdges(const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& from,
        const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& to)
{
    return getGraph()->getEdges<RoleInfoWeightedEdge>(from, to);
}

void Solution::save(const std::string& filename) const
{
    mSpaceTimeNetwork.save(filename);
}

Solution Solution::fromFile(const std::string& filename,
        const moreorg::OrganizationModel::Ptr& om)
{
    SpaceTime::Network network = SpaceTime::Network::fromFile(filename);
    return Solution(network, om);
}


void Solution::narrowMission(Mission& m) const
{
    namespace pa = temporal::point_algebra;
    namespace sym = templ::symbols::constants;

    // Add requirements
    pa::TimePoint::PtrList timepoints = getTimepoints();
    for(size_t t = 0; t < timepoints.size(); ++t)
    {
        if(t != 0)
        {
            pa::QualitativeTimePointConstraint::Ptr qtpc =
                make_shared<pa::QualitativeTimePointConstraint>(timepoints[t-1],
                        timepoints[t], pa::QualitativeTimePointConstraint::Less);
            m.addConstraint(qtpc, false);
        }
    }

    m.prepareTimeIntervals();
    pa::TimePointComparator timepointComparator(m.getTemporalConstraintNetwork());

    for(const Role& role : getAgentRoles())
    {
        SpaceTime::Route route = getRoute(role);
        std::vector<SpaceTime::SpaceIntervalTuple> affectedSpaceIntervals;

        for(size_t i = 1; i < route.size(); ++i)
        {
            sym::Location::Ptr location = route[i]->first();
            pa::TimePoint::Ptr timepoint = route[i]->second();

            SpaceTime::RoleInfoSpaceTimeTuple::Ptr roleInfoTuple =
                mSpaceTimeNetwork.tupleByKeys(location, timepoint);

            size_t cardinality =
                roleInfoTuple->getModelPool({RoleInfo::ASSIGNED})[role.getModel()];

            // Check for same location
            if(location == route[i-1]->first())
            {
                pa::TimePoint::Ptr fromTimepoint = route[i-1]->second();
                pa::TimePoint::Ptr toTimepoint = timepoint;

                SpaceTime::RoleInfoSpaceTimeTuple::Ptr fromRoleInfoTuple =
                    mSpaceTimeNetwork.tupleByKeys(location, fromTimepoint);

                size_t fromCardinality =
                    fromRoleInfoTuple->getModelPool({RoleInfo::ASSIGNED})[role.getModel()];
                size_t stableCardinality = std::min(fromCardinality, cardinality);

                SpaceTime::SpaceIntervalTuple sit(location,
                        solvers::temporal::Interval(
                            fromTimepoint,
                            toTimepoint,
                            timepointComparator)
                );

                // Make sure space timepoint can be referenced
                try {
                    Constraint::Ptr c = m.addResourceLocationCardinalityConstraint(location,
                            fromTimepoint,
                            toTimepoint,
                            role.getModel(),
                            stableCardinality,
                            owlapi::model::OWLCardinalityRestriction::MIN
                            );
                    c->addTag(Constraint::PRIORITY_LOW);
                } catch(const std::invalid_argument& e)
                {
                    // ignore rundant addition
                }

                affectedSpaceIntervals.push_back(sit);
            }

            if(i >= route.size() - 1 || location != route[i+1]->first())
            {
                SpaceTime::SpaceIntervalTuple sit(location,
                        solvers::temporal::Interval(
                            timepoint,
                            timepoint,
                            timepointComparator)
                );

                // Make sure space timepoint can be referenced
                try {
                    Constraint::Ptr c = m.addResourceLocationCardinalityConstraint(location,
                            timepoint,
                            timepoint,
                            role.getModel(),
                            cardinality, // placeholder requirement for reference only
                            owlapi::model::OWLCardinalityRestriction::MIN
                            );
                    c->addTag(Constraint::PRIORITY_LOW);
                } catch(const std::invalid_argument& e)
                {
                    // ignore rundant addition
                }

                affectedSpaceIntervals.push_back(sit);
            }
        }

        if(affectedSpaceIntervals.empty())
        {
            continue;
        }

        constraints::ModelConstraint::Ptr modelConstraint =
            make_shared<constraints::ModelConstraint>(constraints::ModelConstraint::MIN_EQUAL,
                role.getModel(),
                affectedSpaceIntervals,
                1);
        modelConstraint->addTag(Constraint::PRIORITY_LOW);
        m.addConstraint(modelConstraint);
    }
}

Mission::Ptr Solution::toMission(const moreorg::OrganizationModel::Ptr& om,
        const std::string& name
        ) const
{
    Mission::Ptr m = make_shared<Mission>(om, name);

    namespace pa = temporal::point_algebra;
    namespace sym = templ::symbols::constants;

    for(const auto& location : getLocations())
    {
        m->addConstant(location);
    }

    moreorg::Agent agent( getAgentRoles() );
    m->setAvailableResources(agent.getType());

    narrowMission(*m.get());
    return m;
}

Mission::Ptr Solution::getNarrowedMission(const Mission& existingMission) const
{
    Mission::Ptr m = make_shared<Mission>(existingMission);
    narrowMission(*m.get());
    return m;
}

SpaceTime::RoleInfoSpaceTimeTuple::PtrList Solution::getPath(const Role& role)
{
    using namespace solvers::temporal;
    using namespace symbols::constants;
    point_algebra::TimePoint::PtrList timepoints = getTimepoints();
    const point_algebra::TimePoint::Ptr& startingTimepoint = timepoints.front();

    SpaceTime::Network::tuple_t::Ptr startTuple;

    // Find the start point of a role
    for(const symbols::constants::Location::Ptr& location : getLocations())
    {
        try {
            SpaceTime::Network::tuple_t::Ptr tuple = mSpaceTimeNetwork.tupleByKeys(location, startingTimepoint);

            Role::Set assignedRoles = tuple->getRoles(RoleInfo::ASSIGNED);
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
        throw std::runtime_error("templ::solvers::Solution::getPath: Could not find start tuple for role " + role.toString() + ", solution seems to be incomplete");
    }

    // Finding the starting tuple
    using namespace graph_analysis::algorithms;
    // use SpaceTime::Network, which contains information on role for each edge
    // after update from the flow graph
    // foreach role -- find starting point and follow path
    PathConstructor::Ptr pathConstructor =
        make_shared<PathConstructor>(role,
                RoleInfo::TagTxt[ RoleInfo::ASSIGNED ]);
    Skipper skipper = std::bind(&PathConstructor::isInvalidTransition, pathConstructor,std::placeholders::_1);
    DFS dfs(mSpaceTimeNetwork.getGraph(), pathConstructor, skipper);
    dfs.run(startTuple);

    std::vector<graph_analysis::Vertex::Ptr> path = pathConstructor->getPath();
    SpaceTime::RoleInfoSpaceTimeTuple::PtrList typedPath;
    for(const graph_analysis::Vertex::Ptr& v : path)
    {
        SpaceTime::RoleInfoSpaceTimeTuple::Ptr tuple = dynamic_pointer_cast<SpaceTime::RoleInfoSpaceTimeTuple>(v);
        if(!tuple)
        {
            throw std::invalid_argument("templ::solvers::Solution::getPath: could not cast vertex to RoleInfoSpaceTimeTuple");
        }
        typedPath.push_back(tuple);
    }
    return typedPath;
}

} // end namespace solvers
} // end namespace templ
