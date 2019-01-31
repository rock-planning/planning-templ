#include "SolutionAnalysis.hpp"
#include "../RoleInfoVertex.hpp"
#include "../RoleInfoTuple.hpp"
#include "../utils/PathConstructor.hpp"
#include "Cost.hpp"

#include <fstream>
#include <organization_model/Algebra.hpp>
#include <graph_analysis/algorithms/DFS.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <organization_model/facades/Robot.hpp>

using namespace graph_analysis;
using namespace organization_model;

namespace templ {
namespace solvers {

SolutionAnalysis::SolutionAnalysis()
    : mAlpha(1.0)
    , mBeta(1.0)
    , mSigma(1.0)
    , mCost(0.0)
    , mTimeHorizonInS(0)
    , mSafety(0.0)
    , mEfficacy(0.0)
    , mTraveledDistance(0)
    , mReconfigurationCost(0)
    , mTotalNumberOfAgents(0)
    , mNumberOfMobileAgents(0)
    , mAnalyser(organization_model::OrganizationModelAsk())
{}

SolutionAnalysis::SolutionAnalysis(const Mission::Ptr& mission,
        const SpaceTime::Network& solution,
        qxcfg::Configuration configuration)
    : mpMission(mission)
    , mSolutionNetwork(solution)
    , mTimepointComparator(mission->getTemporalConstraintNetwork())
    , mAlpha(1.0)
    , mBeta(1.0)
    , mSigma(1.0)
    , mCost(0.0)
    , mTimeHorizonInS(0)
    , mSafety(0.0)
    , mEfficacy(0.0)
    , mTraveledDistance(0)
    , mReconfigurationCost(0)
    , mTotalNumberOfAgents(0)
    , mNumberOfMobileAgents(0)
    , mAsk(mpMission->getOrganizationModelAsk())
    , mAnalyser(mAsk)
    , mConfiguration(configuration)
{
    mResourceRequirements = Mission::getResourceRequirements(mpMission);

    mAlpha =
        mConfiguration.getValueAs<double>("TransportNetwork/search/options/cost-function/efficacy/weight", 1.0);
    mBeta =
        mConfiguration.getValueAs<double>("TransportNetwork/search/options/cost-function/efficiency/weight", 1.0);
    mSigma =
        mConfiguration.getValueAs<double>("TransportNetwork/search/options/cost-function/safety/weight", 1.0);
}

SolutionAnalysis::SolutionAnalysis(const Mission::Ptr& mission,
        const BaseGraph::Ptr& graph,
        qxcfg::Configuration configuration)
    : mpMission(mission)
    , mTimepointComparator(mission->getTemporalConstraintNetwork())
    , mAlpha(1.0)
    , mBeta(1.0)
    , mSigma(1.0)
    , mCost(0.0)
    , mTimeHorizonInS(0)
    , mSafety(0.0)
    , mEfficacy(0.0)
    , mTraveledDistance(0)
    , mReconfigurationCost(0)
    , mAsk(mpMission->getOrganizationModelAsk())
    , mAnalyser(mAsk)
    , mConfiguration(configuration)

{
    mSolutionNetwork = SpaceTime::Network::fromGraph(graph, mission->getLocations(), mission->getTimepoints());

    mResourceRequirements = Mission::getResourceRequirements(mpMission);

    mAlpha =
        mConfiguration.getValueAs<double>("TransportNetwork/search/options/cost-function/efficacy/weight", 1.0);
    mBeta =
        mConfiguration.getValueAs<double>("TransportNetwork/search/options/cost-function/efficiency/weight", 1.0);
    mSigma =
        mConfiguration.getValueAs<double>("TransportNetwork/search/options/cost-function/safety/weight", 1.0);
}

void SolutionAnalysis::updateAnalyser()
{
    // Adding requirements to analyser
    std::vector<solvers::FluentTimeResource>::const_iterator cit = mResourceRequirements.begin();
    for(; cit != mResourceRequirements.end(); ++cit)
    {
        const solvers::FluentTimeResource& ftr = *cit;
        Resource::Set resources = ftr.getRequiredResources();
        ModelPool agentPool = mAsk.allowSubclasses(ftr.getMinCardinalities(), vocabulary::OM::Actor());

        RequirementSample sample(resources,
                agentPool,
                ftr.getLocation()->getPosition(),
                ftr.getLocation()->getPosition(),
                mTimeAssignment[ ftr.getInterval().getFrom() ],
                mTimeAssignment[ ftr.getInterval().getTo() ]);

        mAnalyser.add(sample);
    }

    // Adding current solution information to analyser
    BaseGraph::Ptr graph = mSolutionNetwork.getGraph();
    EdgeIterator::Ptr edgeIt = graph->getEdgeIterator();
    while(edgeIt->next())
    {
        SpaceTime::Network::edge_t::Ptr edge = dynamic_pointer_cast<SpaceTime::Network::edge_t>(edgeIt->current());
        if(!edge)
        {
            throw std::invalid_argument("SolutionAnalysis: encountered edge type: " + edgeIt->current()->getClassName()
                    + " Are you loading final plan instead of the transport network solution?");
        }

        SpaceTime::Network::tuple_t::Ptr fromTuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(edge->getSourceVertex());
        SpaceTime::Network::tuple_t::Ptr toTuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(edge->getTargetVertex());

        activity::Type activityType = activity::BUSY;
        Agent agent;
        Agent fromAgent(fromTuple->getRoles(RoleInfo::ASSIGNED));

        if(fromTuple->first()->getPosition() != toTuple->first()->getPosition())
        {
            activityType = activity::TRANSPORT;

            // Identify the agent which performs the transition/or
            Agent toAgent(toTuple->getRoles(RoleInfo::ASSIGNED));
            agent = Agent( fromAgent.getIntersection(toAgent) );
        } else {
            agent = fromAgent;
        }

        StatusSample status(agent,
                fromTuple->first()->getPosition(),
                toTuple->first()->getPosition(),
                mTimeAssignment[ fromTuple->second() ],
                mTimeAssignment[ toTuple->second() ],
                Agent::OPERATIVE,
                activityType);
        mAnalyser.add(status);
    }
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

double SolutionAnalysis::getSafety(const FluentTimeResource& ftr, const SpaceTime::Network::tuple_t::Ptr& tuple) const
{
    using namespace organization_model;
    ModelPool minRequired = getMinResourceRequirements(ftr);
    ModelPool minAvailable = getMinAvailableResources(tuple);

    return getSafety(minRequired, minAvailable);
}

double SolutionAnalysis::getSafety(const FluentTimeResource& ftr) const
{
    using namespace organization_model;
    ModelPool minRequired = getMinResourceRequirements(ftr);
    ModelPool minAvailable = getMinAvailableResources(ftr);

    return getSafety(minRequired, minAvailable);
}

double SolutionAnalysis::getSafety(const ModelPool& minRequired, const ModelPool& minAvailable) const
{
    return mAnalyser.getMetric()->computeSharedUse(minRequired, minAvailable);
}

organization_model::ModelPool SolutionAnalysis::getMinAvailableResources(const FluentTimeResource& ftr) const
{
    symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(ftr.getFluent());
    assert(location);

    std::vector<organization_model::ModelPool> availableResources = getAvailableResources(location, ftr.getInterval());

    using namespace organization_model;
    // return the minimum available resources
    // ( min(M_0), min(M_1), ...)
    ModelPool minAvailableResources = organization_model::Algebra::min( availableResources);

    // Infer functionality from this set of resources
    ModelPool functionalities = mAsk.getSupportedFunctionalities(minAvailableResources);
    ModelPool pool = organization_model::Algebra::max(minAvailableResources, functionalities);
    return pool;
}

organization_model::ModelPool SolutionAnalysis::getMinAvailableResources(const SpaceTime::Network::tuple_t::Ptr& tuple) const
{
    using namespace organization_model;
    ModelPool minAvailableResources = getAvailableResources(tuple->first(), tuple->second());

//    // Infer functionality from this set of resources
//    OrganizationModelAsk ask(mpMission->getOrganizationModel(),
//            minAvailableResources,
//            true);
//    // Creating model pool from available functionalities
//    ModelPool functionalities = ask.getSupportedFunctionalities();
//    return organization_model::Algebra::max(minAvailableResources, functionalities);
    return minAvailableResources;
}

organization_model::ModelPool SolutionAnalysis::getMaxAvailableResources(const FluentTimeResource& ftr) const
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

    Role::Set identifiedRoles;

    for(TimePoint::Ptr timepoint : timepoints)
    {
        if( mTimepointComparator.inInterval(timepoint, interval.getFrom(), interval.getTo()) )
        {
            // identified relevant tuple
            try {
                SpaceTime::Network::tuple_t::Ptr tuple = mSolutionNetwork.tupleByKeys(location, timepoint);
                Role::Set foundRoles = tuple->getRoles(RoleInfo::ASSIGNED);
                Role::List roles(foundRoles.begin(), foundRoles.end());

                identifiedRoles.insert(foundRoles.begin(), foundRoles.end());

                organization_model::ModelPool currentPool = Role::getModelPool(roles);
                modelPools.push_back(currentPool);
            } catch(const std::exception& e)
            {
                LOG_WARN_S << e.what();
            }
        }
    }

    // An completely empty model pool does not
    // correctly reflect the minimums, so
    // we have to expand the existing set
    for(ModelPool& pool : modelPools)
    {
        for(const Role& role : identifiedRoles)
        {
            if(pool.end() == pool.find(role.getModel()))
            {
                pool.insert(ModelPool::value_type(role.getModel(), 0));
            }
        }
    }

    return modelPools;
}

organization_model::ModelPool SolutionAnalysis::getAvailableResources(const symbols::constants::Location::Ptr& location,
        const solvers::temporal::point_algebra::TimePoint::Ptr& timepoint) const
{
    ModelPool modelPool;
    // identified relevant tuple
    try {
        SpaceTime::Network::tuple_t::Ptr tuple = mSolutionNetwork.tupleByKeys(location, timepoint);
        Role::Set foundRoles = tuple->getRoles(RoleInfo::ASSIGNED);
        Role::List roles(foundRoles.begin(), foundRoles.end());
        modelPool = Role::getModelPool(roles);
    } catch(const std::exception& e)
    {
        LOG_WARN_S << e.what();
    }
    return modelPool;
}

SolutionAnalysis::MinMaxModelPools SolutionAnalysis::getRequiredResources(const symbols::constants::Location::Ptr& location, const solvers::temporal::Interval& interval) const
{
    using namespace temporal::point_algebra;
    MinMaxModelPools minMaxModelPools;

    std::vector<solvers::FluentTimeResource>::const_iterator cit = mResourceRequirements.begin();
    for(; cit != mResourceRequirements.end(); ++cit)
    {
        const solvers::FluentTimeResource& ftr = *cit;

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
            minMaxModelPools.first.push_back( ftr.getMinCardinalities() );
            minMaxModelPools.second.push_back( ftr.getMaxCardinalities() );
        }
    }

    return minMaxModelPools;
}

SolutionAnalysis::MinMaxModelPools SolutionAnalysis::getRequiredResources(const FluentTimeResource& ftr) const
{
    using namespace temporal::point_algebra;
    MinMaxModelPools minMaxModelPools;

    std::vector<solvers::FluentTimeResource>::const_iterator cit = mResourceRequirements.begin();
    for(; cit != mResourceRequirements.end(); ++cit)
    {
        const solvers::FluentTimeResource& requirementFtr = *cit;
        if(ftr.getLocation() == requirementFtr.getLocation() &&
                ftr.getInterval() == requirementFtr.getInterval())
        {
            ModelPool minCardinalities = ftr.getMinCardinalities();
            for(const organization_model::Resource& resource : ftr.getRequiredResources())
            {
                // Assuming the functionalities have max
                if(mAsk.ontology().isSubClassOf(resource.getModel(),
                            vocabulary::OM::Functionality()))
                {
                    minCardinalities[resource.getModel()] = 1;
                }
            }

            minMaxModelPools.first.push_back(minCardinalities);
            minMaxModelPools.second.push_back(ftr.getMaxCardinalities());
            return minMaxModelPools;
        }
    }

    throw std::invalid_argument("templ::solvers::SolutionAnalysis::getRequiredResources: could not find the corrensponding requirement for an existing fluent time resource");
}

void SolutionAnalysis::propagateTemporalConstraints()
{
    mPlan = computePlan();
    computeReconfigurationCost();
    quantifyTime();
}

void SolutionAnalysis::analyse()
{
    propagateTemporalConstraints();

    mTimepoints = mSolutionNetwork.getTimepoints();

    //updateAnalyser();
    // a collect all requirements of the mission -- as translated from the
    // persistence conditions
    if(mAlpha > 0)
    {
        computeEfficacy();
    }
    if(mBeta > 0)
    {
        computeEfficiency();
    }
    if(mSigma > 0)
    {
        computeSafety();
    }
    computeAgentCount();

    mCost = mAlpha*mEfficacy + mBeta*mEfficiency + mSigma*mSafety;
}

void SolutionAnalysis::save(const std::string& _filename) const
{
    std::string planFilename = _filename;
    if(planFilename.empty())
    {
        planFilename = mpMission->getLogger()->filename("final_plan.gexf");
    }
    graph_analysis::io::GraphIO::write(planFilename, mPlan.getGraph());

    std::string solutionNetworkFilename = _filename;
    if(solutionNetworkFilename.empty())
    {
        solutionNetworkFilename = mpMission->getLogger()->filename("final_solution_network.gexf");
    }
    graph_analysis::io::GraphIO::write(solutionNetworkFilename, mSolutionNetwork.getGraph());

    // stats to string
    std::string filename = mpMission->getLogger()->getBasePath() +
        "solution_analysis.log";
    std::ofstream outfile;
    if(!boost::filesystem::exists(filename))
    {
        outfile.open(filename);
        outfile << getRowDescriptor() << std::endl;
        outfile << toRow() << std::endl;
    } else {
        outfile.open(filename, std::ios_base::app);
        outfile << toRow() << std::endl;
    }
}

void SolutionAnalysis::saveRow(const std::string& filename, size_t sessionId) const
{
    std::ofstream outfile;
    if(!boost::filesystem::exists(filename))
    {
        outfile.open(filename);
        outfile << getRowDescriptor() << std::endl;
        outfile << toRow(sessionId) << std::endl;
    } else {
        outfile.open(filename, std::ios_base::app);
        outfile << toRow(sessionId) << std::endl;
    }
}

void SolutionAnalysis::saveModelPool(const std::string& filename) const
{
    std::ofstream outfile;
    outfile.open(filename);
    ModelPool modelPool = Role::getModelPool(mRoles);
    for(const ModelPool::value_type& value : modelPool)
    {
        outfile << value.first.toString();
        outfile << " ";
        outfile << value.second;
        outfile << std::endl;
    }
    outfile.close();
}

std::string SolutionAnalysis::getRowDescriptor() const
{
    std::stringstream ss;
    ss << "# ";
    ss << " [session-id] ";
    ss << " [alpha] [beta] [sigma] ";
    ss << " [efficacy] [efficiency in kWh] [safety] ";
    ss << " [timehorizon in s] [travel-distance in m] [reconfiguration-cost in s]";
    ss << " [number of agents] [number of mobile agents]";
    return ss.str();
}

std::string SolutionAnalysis::toRow(size_t sessionId) const
{
    std::stringstream ss;
    if(sessionId != 0)
    {
        ss << sessionId << " ";
    } else {
        ss << mpMission->getLogger()->getSessionId() << " ";
    }
    ss << mAlpha << " ";
    ss << mBeta << " ";
    ss << mSigma << " ";
    ss << mEfficacy << " ";
    ss << mEfficiency << " ";
    ss << mSafety << " ";
    ss << mTimeHorizonInS << " ";
    ss << mTraveledDistance << " ";
    ss << mReconfigurationCost << " ";
    ss << mTotalNumberOfAgents << " ";
    ss << mNumberOfMobileAgents << " ";
    return ss.str();
}


bool SolutionAnalysis::isStartDepotRequirement(const FluentTimeResource& ftr) const
{
    return ftr.getInterval().getFrom()->equals(mTimepoints.front());
}

double SolutionAnalysis::degreeOfFulfillment(const solvers::FluentTimeResource& ftr)
{
    // Ignore start requirements
    if(isStartDepotRequirement(ftr))
    {
        return 1.0;
    }

    SpaceTime::Network::tuple_t::PtrList tuples = getTuples(ftr);
    for(const SpaceTime::Network::tuple_t::Ptr& roleInfo : tuples)
    {
        std::set<Role> assigned = roleInfo->getRoles(RoleInfo::ASSIGNED);
        std::set<Role> required = roleInfo->getRoles(RoleInfo::REQUIRED);

        if(!std::includes(assigned.begin(), assigned.end(), required.begin(),
                required.end()))
        {
            return 0.0;
        }
    }
    return 1.0;
}

organization_model::ModelPool SolutionAnalysis::getMinResourceRequirements(const FluentTimeResource& ftr) const
{
    using namespace organization_model;
    return getRequiredResources(ftr).first.front();
}

organization_model::ModelPool SolutionAnalysis::getMaxResourceRequirements(const FluentTimeResource& ftr) const
{
    using namespace organization_model;
    return getRequiredResources(ftr).second.front();
}

SpaceTime::Network::tuple_t::Ptr SolutionAnalysis::getFromTuple(const FluentTimeResource& ftr) const
{
    return mSolutionNetwork.tupleByKeys(ftr.getLocation(), ftr.getInterval().getFrom());
}

SpaceTime::Network::tuple_t::Ptr SolutionAnalysis::getToTuple(const FluentTimeResource& ftr) const
{
    return mSolutionNetwork.tupleByKeys(ftr.getLocation(), ftr.getInterval().getTo());
}

SpaceTime::Network::tuple_t::PtrList SolutionAnalysis::getTuples(const
        FluentTimeResource& ftr) const
{
    return mSolutionNetwork.getTuples(ftr.getInterval().getFrom(),
            ftr.getInterval().getFrom(), ftr.getLocation());
}

organization_model::ModelPoolDelta SolutionAnalysis::getMinMissingResourceRequirements(const solvers::FluentTimeResource& ftr) const
{
    ModelPool requiredResources = getMinResourceRequirements(ftr);
    ModelPool maxAvailableResources = getMinAvailableResources(ftr);

    // Creating model pool from available functionalities
    ModelPool functionalities = mAsk.getSupportedFunctionalities(maxAvailableResources);
    ModelPool availableResources = organization_model::Algebra::max(maxAvailableResources, functionalities);

    return Algebra::delta(requiredResources, availableResources);
}

organization_model::ModelPoolDelta SolutionAnalysis::getMaxMissingResources(const solvers::FluentTimeResource& ftr) const
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
        RoleInfoVertex::Ptr roleInfo = make_shared<RoleInfoVertex>();
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
        std::set<Role> roles = roleInfo->getRoles(RoleInfo::ASSIGNED);
        if(roles.empty())
        {
            continue;
        }
        for(const Role& role : roles)
        {
            Edge::Ptr edge = make_shared<Edge>("requires");
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
        std::set<Role> roles = roleInfo->getRoles(RoleInfo::ASSIGNED);
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

        HyperEdge::Ptr hyperEdge = make_shared<HyperEdge>(vertices, edgeLabel.str());
        hyperGraph->addHyperEdge(hyperEdge);

        for(const Role& role : roles)
        {
            Edge::Ptr edge = make_shared<Edge>("requires");
            edge->setSourceVertex(hyperEdge);
            RoleInfoVertex::Ptr targetRoleVertex = role2VertexMap[role];
            edge->setTargetVertex(targetRoleVertex);
            hyperGraph->addEdge(edge);
        }
    }
    return hyperGraph;
}

void SolutionAnalysis::quantifyTime()
{
    using namespace solvers::temporal;
    TemporalConstraintNetwork tcn;
    Cost cost(mpMission->getOrganizationModelAsk());

    double travelDistanceInM = 0.0;

    // Apply constraints from the current solution
    // TODO: space time network: iterator over all 'solution' edges
    using namespace graph_analysis;
    graph_analysis::EdgeIterator::Ptr edgeIt = mSolutionNetwork.getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {

        SpaceTime::Network::edge_t::Ptr edge = dynamic_pointer_cast<SpaceTime::Network::edge_t>( edgeIt->current() );
        if(!edge)
        {
            throw std::invalid_argument("SolutionAnalysis: encountered edge type: " + edgeIt->current()->getClassName()
                    + " Are you loading final plan instead of the transport network solution?");
        }

        Role::Set roles = edge->getRoles(RoleInfo::ASSIGNED);
        if(roles.empty())
        {
            continue;
        }

        SpaceTime::Network::tuple_t::Ptr sourceTuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(edge->getSourceVertex());
        SpaceTime::Network::tuple_t::Ptr targetTuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(edge->getTargetVertex());

        symbols::constants::Location::Ptr sourceLocation = sourceTuple->first();
        symbols::constants::Location::Ptr targetLocation = targetTuple->first();

        double distanceInM = cost.getTravelDistance({sourceLocation, targetLocation});
        double minTravelTime = cost.estimateTravelTime(sourceLocation, targetLocation, roles);
        LOG_DEBUG_S << "Estimated travelTime: " << minTravelTime << " for " << Role::toString(roles)
            << "    from: " << sourceLocation->toString() << "/" << sourceTuple->second()->toString() << std::endl
            << "    to: " << targetLocation->toString() << "/" << targetTuple->second()->toString() << std::endl
            << "    estimated travelDistance in m: " << distanceInM;

        travelDistanceInM += distanceInM;

        point_algebra::TimePoint::Ptr sourceTimepoint = sourceTuple->second();
        point_algebra::TimePoint::Ptr targetTimepoint = targetTuple->second();

        IntervalConstraint::Ptr intervalConstraint =
            make_shared<IntervalConstraint>(sourceTimepoint, targetTimepoint);

        // Compute the required transition time
        RoleInfo::Ptr roleInfo = dynamic_pointer_cast<RoleInfo>(sourceTuple);
        double reconfigurationCost = 0;
        try {
            reconfigurationCost = roleInfo->getAttribute(RoleInfo::RECONFIGURATION_COST);
        } catch(const std::exception& e)
        {
            LOG_WARN_S << "No reconfiguration cost for " << roleInfo->toString(4);
        }

        double requiredTime = minTravelTime + reconfigurationCost;
        if(requiredTime > 0)
        {
            Bounds bounds(requiredTime, std::numeric_limits<double>::max());
            intervalConstraint->addInterval(bounds);
            tcn.addIntervalConstraint(intervalConstraint);

            LOG_DEBUG_S << "Interval constraint between: " << sourceTimepoint->toString() << " and " << targetTimepoint->toString() << " min travel time: "
                << minTravelTime;
        }
    }

    for(Constraint::Ptr c : mpMission->getConstraints())
    {
        IntervalConstraint::Ptr ic = dynamic_pointer_cast<IntervalConstraint>(c);
        if(ic)
        {
            Edge::PtrList edges = tcn.getDistanceGraph()->getEdges(ic->getSourceVertex(), ic->getTargetVertex());
            if(edges.empty())
            {
                tcn.addIntervalConstraint(ic);
            } else {
                IntervalConstraint::Ptr existingIc = dynamic_pointer_cast<IntervalConstraint>(edges.front());
                existingIc->appendBounds(*ic);
            }
        }
    }

    tcn.stpWithConjunctiveIntervals();
    tcn.stp();
    tcn.minNetwork();

    mTimeAssignment = TemporalConstraintNetwork::getAssignment(tcn.getDistanceGraph(), mSolutionNetwork.getTimepoints());
    mTimeHorizonInS = TemporalConstraintNetwork::getTimeHorizon(mTimeAssignment);
    mTraveledDistance = travelDistanceInM;
}

void SolutionAnalysis::computeSafety(bool ignoreStartDepot)
{
    mSafety = 1.0;
    temporal::point_algebra::TimePoint::PtrList timepoints = mSolutionNetwork.getTimepoints();
    for(const FluentTimeResource& ftr : mResourceRequirements)
    {
        SpaceTime::Network::tuple_t::PtrList tuples = mSolutionNetwork.getTuples(ftr.getInterval().getFrom(),
                ftr.getInterval().getTo(),
                ftr.getLocation());

        for(const SpaceTime::Network::tuple_t::Ptr& tuple : tuples)
        {
            double value = 1.0;
            // Ignore redundancy at starting depot
            if(!isStartDepotRequirement(ftr) && ignoreStartDepot)
            {
                try {
                    value = getSafety(ftr, tuple);
                    mSafety = std::min(value, mSafety);
                    LOG_INFO_S << "Metric: " << value << std::endl
                        << "    at: " << tuple->first()->toString() << std::endl;

                } catch(const std::exception& e)
                {
                    if(isStartDepotRequirement(ftr))
                    {
                        LOG_INFO_S << "Metric: not all resources from start depot used:" << std::endl
                            << "    at: " << ftr.getLocation()->toString() << std::endl
                            << "    over: " <<  std::endl
                            << ftr.getInterval().toString(8) << std::endl
                            << e.what();

                        ModelPool minAvailable = getMinAvailableResources(ftr);
                        ModelPool agentPool = mpMission->getOrganizationModelAsk().allowSubclasses(minAvailable, vocabulary::OM::Actor());
                        if(!minAvailable.empty())
                        {
                            value = getSafety(agentPool, agentPool);
                        } else {
                            value = 0;
                        }

                    } else {
                        LOG_WARN_S << "Metric: requirements not fulfilled" << std::endl
                            << "    at: " << ftr.getLocation()->toString() << std::endl
                            << "    over: " <<  std::endl
                            << ftr.getInterval().toString(8) << std::endl
                            << ftr.toString(8) << std::endl
                            << tuple->toString() << std::endl
                            << e.what();
                        value = 0.0;
                    }
                }
            }

            tuple->setAttribute(RoleInfo::SAFETY, value);
        }
    }

    VertexIterator::Ptr vertexIt = mSolutionNetwork.getGraph()->getVertexIterator();
    while(vertexIt->next())
    {
        RoleInfo::Ptr roleInfo = dynamic_pointer_cast<RoleInfo>(vertexIt->current());
        if(!roleInfo->hasAttribute(RoleInfo::SAFETY))
        {
            ModelPool modelPool = Role::getModelPool( roleInfo->getAllRoles() );
            if(!modelPool.empty())
            {
                double value = getSafety(modelPool, modelPool);
                roleInfo->setAttribute(RoleInfo::SAFETY, value);
            }
        }
    }
}



Plan SolutionAnalysis::computePlan() const
{
    Plan plan(mpMission);

    using namespace solvers::temporal;
    using namespace symbols::constants;
    point_algebra::TimePoint::PtrList timepoints = mpMission->getTimepoints();
    Location::PtrList locations = mpMission->getLocations();

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
            LOG_WARN_S << "Could not find start tuple for role " + role.toString() << " solution seems to be incomplete";
            continue;
        }

        // Finding the starting tuple
        using namespace graph_analysis::algorithms;
        // use SpaceTime::Network, which contains information on role for each edge
        // after update from the flow graph
        // foreach role -- find starting point and follow path
        PathConstructor::Ptr pathConstructor =
            make_shared<PathConstructor>(role,
                    RoleInfo::TagTxt[ RoleInfo::ASSIGNED ]);
        Skipper skipper = boost::bind(&PathConstructor::isInvalidTransition, pathConstructor,_1);
        DFS dfs(mSolutionNetwork.getGraph(), pathConstructor, skipper);
        dfs.run(startTuple);

        std::vector<graph_analysis::Vertex::Ptr> path = pathConstructor->getPath();
        path.insert(path.begin(), startTuple);
        plan.add(role,path);
    }

    graph_analysis::Vertex::PtrList openRequirements;
    for(const FluentTimeResource& ftr : mResourceRequirements)
    {
        const Interval& i = ftr.getInterval();
        try {
            SpaceTime::Network::tuple_t::Ptr tuple = mSolutionNetwork.tupleByKeys(ftr.getLocation(),
                    i.getFrom());
            openRequirements.push_back(tuple);
        } catch(const std::invalid_argument& e)
        {
            LOG_WARN_S << "Failed to retrieve key for: " << ftr.getLocation()->toString() << " and " << i.getFrom()->toString();
        }
    }

    return plan;
}

void SolutionAnalysis::computeAgentCount()
{
    std::set<Role> roles;
    for(const solvers::FluentTimeResource& ftr : mResourceRequirements)
    {
        if(isStartDepotRequirement(ftr))
        {
            SpaceTime::Network::tuple_t::PtrList tuples = getTuples(ftr);
            for(const SpaceTime::Network::tuple_t::Ptr& roleInfo : tuples)
            {
                std::set<Role> assigned = roleInfo->getRoles(RoleInfo::ASSIGNED);
                roles.insert(assigned.begin(), assigned.end());
                break;
            }
        }
    }

    mRoles = roles;
    mTotalNumberOfAgents = 0;
    mNumberOfMobileAgents = 0;
    for(const Role& role : roles)
    {
        facades::Robot robot = facades::Robot::getInstance(role.getModel(), mpMission->getOrganizationModelAsk());
        if(robot.isMobile())
        {
            ++mNumberOfMobileAgents;
        }
        ++mTotalNumberOfAgents;
    }
}

void SolutionAnalysis::computeEfficacy()
{
    double fulfillment = 0.0;
    for(const solvers::FluentTimeResource& ftr : mResourceRequirements)
    {
        fulfillment += degreeOfFulfillment(ftr);
    }
    mEfficacy = fulfillment*1.0/mResourceRequirements.size();
}

void SolutionAnalysis::computeEfficiency()
{
    mEfficiency = 0.0;


    const Plan::RoleBasedPlan& plan = mPlan.getRoleBasedPlan();
    for(const Plan::RoleBasedPlan::value_type& v : plan)
    {
        const Role& role = v.first;
        if(role == Role())
        {
            continue;
        }
        //const Vertex::PtrList& plan = v.second;
        facades::Robot robot = facades::Robot::getInstance(role.getModel(), mpMission->getOrganizationModelAsk());
        double efficiencyInkWh = robot.estimatedEnergyCostFromTime(mTimeHorizonInS)/(3600*1000.0);

        mEfficiencyPerRole[role] = efficiencyInkWh;
        mEfficiency += efficiencyInkWh;
    }
}

void SolutionAnalysis::computeReconfigurationCost()
{
    double totalReconfigurationCost = 0.0;
    // TODO: Adapt to mSolutionNetwork -- after annotating the network
    VertexIterator::Ptr vertexIt = mPlan.getGraph()->getVertexIterator();
    while(vertexIt->next())
    {
        Vertex::Ptr vertex = vertexIt->current();
        double reconfigurationCost = 0;

        RoleInfo::Ptr roleInfo = dynamic_pointer_cast<RoleInfo>(vertex);
        if(!roleInfo)
        {
            throw std::runtime_error("templ::solvers::SolutionAnalysis::computeReconfigurationCost: failed to cast to RoleInfo."
                    " Class of node is " + vertex->getClassName());
        } else if(!roleInfo->getAllRoles().empty())
        {
            try {
                reconfigurationCost = computeReconfigurationCost(vertex, mPlan.getGraph());
            } catch(const std::exception& e)
            {
                // handle unfulfilled requirements
            }
            roleInfo->setAttribute(RoleInfo::RECONFIGURATION_COST, reconfigurationCost);
        }

        totalReconfigurationCost += reconfigurationCost;
    }

    mReconfigurationCost = totalReconfigurationCost;
}

double SolutionAnalysis::computeReconfigurationCost(const Vertex::Ptr& vertex, const BaseGraph::Ptr& graph)
{
    RoleInfo::Ptr tuple = dynamic_pointer_cast<RoleInfo>(vertex);

    Agent requirementAgent(tuple->getAllRoles());
    Agent::Set actualRequirement = { requirementAgent };

    Agent::Set in;
    EdgeIterator::Ptr inEdgeIt = graph->getInEdgeIterator(vertex);
    while(inEdgeIt->next())
    {
        Edge::Ptr edge = inEdgeIt->current();
        CapacityLink::Ptr inAgents = dynamic_pointer_cast<CapacityLink>(edge);
        if(inAgents)
        {
            Role::Set allRoles = inAgents->getAllRoles();
            Agent inAgent(allRoles);
            in.insert(inAgent);
        }
    }
    Agent::Set out;
    EdgeIterator::Ptr outEdgeIt = graph->getOutEdgeIterator(vertex);
    while(outEdgeIt->next())
    {
        Edge::Ptr edge = outEdgeIt->current();
        CapacityLink::Ptr outAgents = dynamic_pointer_cast<CapacityLink>(edge);
        if(outAgents)
        {
            Role::Set allRoles = outAgents->getAllRoles();
            Agent outAgent(allRoles);
            out.insert(outAgent);
        }
    }

    double cost = 0.0;
    LOG_INFO_S << "Compute reconfiguration cost at " << vertex->toString()
        << "    from" << Agent::toString(in,4)
        << "    to"  << Agent::toString(out,4)
        << "    requirement"  << Agent::toString(actualRequirement,4);
    try {
        if(!in.empty())
        {
            cost += mAnalyser.getHeuristics().getReconfigurationCost(in, actualRequirement);
            LOG_INFO_S << "Reconfiguration: from: " << Agent::toString(in,4)
                << "to: " << Agent::toString(actualRequirement, 4)
                << "with cost: " << cost;
        }
        if(!out.empty())
        {
            cost += mAnalyser.getHeuristics().getReconfigurationCost(actualRequirement, out);
            LOG_INFO_S << "Reconfiguration: from: " << Agent::toString(actualRequirement,4)
                << "to: " << Agent::toString(out, 4)
                << "with cost: " << cost;
        }
    } catch(const std::invalid_argument& e)
    {
        LOG_WARN_S << "Failed to compute reconfiguration cost at: " << vertex->toString() << e.what();
        throw;
    }
    return cost;
}



std::string SolutionAnalysis::toString(size_t indent) const
{
    std::string hspace(indent, ' ');
    std::stringstream ss;
    ss << hspace << "SolutionAnalysis:" << std::endl;
    ss << hspace << "    Resulting plan:" << std::endl;
    ss << mPlan.toString(indent + 8);
    ss << hspace << "        time horizon: " << mTimeHorizonInS << std::endl;
    for(const temporal::point_algebra::TimePoint::Ptr& tp : mSolutionNetwork.getTimepoints())
    {

        temporal::TemporalConstraintNetwork::Assignment::const_iterator cit = mTimeAssignment.find(tp);
        ss << hspace << "            " << tp->getLabel() << ": " << cit->second << std::endl;
    }
    ss << hspace << "        cost: " << mCost << std::endl;
    ss << hspace << "            efficacy: " << mEfficacy << std::endl;
    ss << hspace << "            efficiency: " << mEfficiency << std::endl;
    ss << hspace << "            reconfiguration: " << mReconfigurationCost << std::endl;
    ss << hspace << "            safety: " << mSafety << std::endl;
    ss << hspace << "            travel distance: " << mTraveledDistance << std::endl;
    ss << hspace << "        # of agents: " << mTotalNumberOfAgents << std::endl;
    ss << hspace << "        # of mobile agents: " << mNumberOfMobileAgents << std::endl;
    ss << Role::getModelPool(mRoles).toString(indent + 12) << std::endl;
    return ss.str();
}

} // end namespace solvers
} // end namespace templ
