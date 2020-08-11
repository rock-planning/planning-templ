#include "Solution.hpp"
#include <limits>
#include <functional>
#include <graph_analysis/algorithms/DFS.hpp>
#include <moreorg/Agent.hpp>
#include <moreorg/facades/Robot.hpp>

#include "../utils/PathConstructor.hpp"
#include "../Mission.hpp"
#include "../utils/PathConstructor.hpp"

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

const SpaceTime::RoleInfoSpaceTimeTuple::Ptr& Solution::getStart(const Role& role) const
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

    temporal::point_algebra::TimePoint::PtrList timepoints = getTimepoints();
    for(size_t t = 0; t < timepoints.size(); ++t)
    {
        if(timepoints[t] == from)
        {
            startIndex = t;
            inRoute = true;
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
            }

            edge->addRole(role, tag);

            if(prevTuple->first() == roleInfoTuple->first())
            {
                edge->setWeight(std::numeric_limits<double>::max());
            } else
            {
                Role::Set allRoles = edge->getRoles({ RoleInfo::AVAILABLE });
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

        if(timepoints[t] == to)
        {
            inRoute = false;
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

Mission::Ptr Solution::toMission(const moreorg::OrganizationModel::Ptr& om,
        const std::string& name
        ) const
{

    namespace pa = temporal::point_algebra;
    namespace sym = templ::symbols::constants;

    Mission::Ptr m = make_shared<Mission>(om, name);

    for(const auto& location : getLocations())
    {
        m->addConstant(location);
    }

    moreorg::Agent agent( getAgentRoles() );
    m->setAvailableResources(agent.getType());

    // Starting requirements
    pa::TimePoint::PtrList timepoints = getTimepoints();
    for(size_t t = 0; t < timepoints.size(); ++t)
    {
        if(t != 0)
        {
            pa::QualitativeTimePointConstraint::Ptr qtpc =
                make_shared<pa::QualitativeTimePointConstraint>(timepoints[t-1],
                        timepoints[t], pa::QualitativeTimePointConstraint::Less);
            m->addConstraint(qtpc);
        }

        for(const auto& location : getLocations())
        {
            LOG_WARN_S << "Process location: " << location->toString();

            SpaceTime::RoleInfoSpaceTimeTuple::Ptr roleInfoTuple =
                mSpaceTimeNetwork.tupleByKeys(location, timepoints[t]);

            moreorg::ModelPool assigedAgentType = roleInfoTuple->getModelPool({ RoleInfo::ASSIGNED });
            for(const auto& pair : assigedAgentType)
            {
                LOG_WARN_S << "Processing Model " << pair.first.toString() << " "
                    << timepoints[t]->getLabel() << "--" << timepoints[t+1]->getLabel();

                if(t == 0)
                {
                    // start assignment requires 'max' constraints
                    m->addResourceLocationCardinalityConstraint(location, timepoints[t],
                            timepoints[t+1],
                            pair.first,  //  model
                            pair.second, // cardinality
                            owlapi::model::OWLCardinalityRestriction::MAX
                            );
                }

                // start assignment requires 'max' constraints
                m->addResourceLocationCardinalityConstraint(location, timepoints[t],
                        timepoints[t+1],
                        pair.first,  //  model
                        pair.second, // cardinality
                        owlapi::model::OWLCardinalityRestriction::MIN
                        );
            }
        }
    }
    return m;
}

Mission::Ptr Solution::toMission(const Mission::Ptr& existingMission) const
{
    throw std::runtime_error("templ::Solution::toMission not implemented");
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
