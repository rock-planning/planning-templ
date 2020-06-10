#include "TransportNetwork.hpp"
#include <base-logging/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include <numeric/Stats.hpp>
#include <gecode/minimodel.hh>
#include <gecode/set.hh>
#include <gecode/gist.hh>
#include <gecode/search.hh>

#include <iterator>
#include <iomanip>
#include <fstream>
#include <Eigen/Dense>

#include <moreorg/Algebra.hpp>
#include <moreorg/vocabularies/OM.hpp>
#include <moreorg/facades/Robot.hpp>

#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/algorithms/LPSolver.hpp>

#include "../../SharedPtr.hpp"
#include "../../symbols/object_variables/LocationCardinality.hpp"
#include "../../SpaceTime.hpp"
#include "ConstraintMatrix.hpp"
#include "branchers/TimelineBrancher.hpp"
#include "propagators/IsPath.hpp"
#include "propagators/InEdgesRestriction.hpp"
#include "propagators/IsValidTransportEdge.hpp"
#include "propagators/MultiCommodityFlow.hpp"
#include "utils/Formatter.hpp"
#include "utils/Converter.hpp"
#include "../../utils/CSVLogger.hpp"
#include "MissionConstraints.hpp"
#include "Search.hpp"
#include "../SolutionAnalysis.hpp"
#include "MissionConstraintManager.hpp"
#include "../../constraints/ModelConstraint.hpp"

using namespace templ::solvers::csp::utils;

namespace templ {
namespace solvers {
namespace csp {

bool TransportNetwork::msInteractive = false;
TransportNetwork::FlowSolutions TransportNetwork::msMinCostFlowSolutions;

std::string TransportNetwork::Solution::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    {
        ModelDistribution::const_iterator cit = mModelDistribution.begin();
        size_t count = 0;
        ss << hspace << "ModelDistribution" << std::endl;
        for(; cit != mModelDistribution.end(); ++cit)
        {
            const FluentTimeResource& fts = cit->first;
            ss << hspace << "--- requirement #" << count++ << std::endl;
            ss << hspace << fts.toString() << std::endl;

            const moreorg::ModelPool& modelPool = cit->second;
            ss << modelPool.toString(indent) << std::endl;
        }
    }
    {
        RoleDistribution::const_iterator cit = mRoleDistribution.begin();
        size_t count = 0;
        ss << hspace << "RoleDistribution" << std::endl;
        for(; cit != mRoleDistribution.end(); ++cit)
        {
            const FluentTimeResource& fts = cit->first;
            ss << hspace << "--- requirement #" << count++ << std::endl;
            ss << hspace << fts.toString() << std::endl;

            const Role::List& roles = cit->second;
            ss << hspace << Role::toString(roles) << std::endl;
        }
    }
    {
        ss << hspace << "Timelines (" << mTimelines.size() << ")" << std::endl;
        ss << hspace  << RoleTimeline::toString(mTimelines, 4);
    }
    return ss.str();
}

SpaceTime::Network TransportNetwork::Solution::toNetwork() const
{
    return SpaceTime::toNetwork(mLocations, mTimepoints,
            RoleTimeline::collectTimelines(mTimelines) );
}

// return true in order to enforce a restart
// return false would continue search without a restart
bool TransportNetwork::master(const Gecode::MetaInfo& mi)
{
    switch(mi.type())
    {
        case Gecode::MetaInfo::RESTART:
            if(mi.last() != NULL)
            {
                constrain(*mi.last());
            }
            mi.nogoods().post(*this);
            return true;
        case Gecode::MetaInfo::PORTFOLIO:
            Gecode::BrancherGroup::all.kill(*this);
            break;
        default:
            break;
    }
    return true;
}

void TransportNetwork::next(const TransportNetwork& lastSpace, const Gecode::MetaInfo& mi)
{
    breakpoint("BEGIN next()");

    // constrain the next space // but not the first
    if(mi.last() != NULL)
    {
        constrainSlave(*mi.last());

        // the last space is the result of the 'first' slave so this
        // is our basic (master) solution to start from
        mMinCostFlowFlaws = lastSpace.mMinCostFlowFlaws;
        mFlawResolution = lastSpace.mFlawResolution;
        mRequiredResolutionOptions = lastSpace.mRequiredResolutionOptions;
    }

    breakpointStart()
        << "next():" << std::endl
        << "    # flaws: " << mMinCostFlowFlaws.size() << std::endl
        << "    # resolution options: " <<
        mFlawResolution.remainingDraws().size() << std::endl
        ;
    breakpointEnd();

    namespace ga = graph_analysis::algorithms;

    Constraint::PtrList constraints = FlawResolution::selectBestResolution(*this, lastSpace, lastSpace.cost().val(), mFlawResolution.getResolutionOptions());
    if(constraints.empty())
    {
        std::cout << "    # no applicable resolvers better than " << lastSpace.cost().val() <<
            "-- failing search" << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

        this->fail();
        return;
    }

    //FlawResolution::ResolutionOptions resolutionOptions = mFlawResolution.current();
    // Update the existing set of resolution options
    // this set will be propagated to any clones (and thus slaves that will
    // improve any solution of this)
    //
    // Now assuming we only improve an existing solution, but not to a perfect
    // solution then what?
    //
    // Goal: we want to record the benefit of fixing a flaw for a 'master'
    // solution

    for(const Constraint::Ptr& constraint : constraints)
    {
        addConstraint(constraint, *this);
    }
    // Recompute unary resource usage since constraints have changed
    enforceUnaryResourceUsage();
}

// Constrain function is called with the last solution found as argument (if a
// solution has already been found
void TransportNetwork::constrain(const Gecode::Space& lastSpace)
{
    const TransportNetwork& lastTransportNetwork = static_cast<const TransportNetwork&>(lastSpace);

    breakpointStart()
        << "constrain()" << std::endl
        << "Last state: " << std::endl
        << "    # cost: "<< lastTransportNetwork.mCost.val() << std::endl
        << "    # flaws: "<< lastTransportNetwork.mMinCostFlowFlaws.size() << std::endl
        << "    # resolution options: " << lastTransportNetwork.mFlawResolution.remainingDraws().size() <<
        std::endl
        << "Current: " << std::endl
        << "    # cost: " << cost() << std::endl;
    breakpointEnd();


    bool hillClimbing = mConfiguration.getValueAs<bool>("TransportNetwork/search/options/hill-climbing",false);
    if(hillClimbing)
    {
        rel(*this, cost(), Gecode::IRT_LE, lastTransportNetwork.cost().val());
        // TODO:
        // check on the number of flows and the cost of the solution in order to
        // find an improved solution
    }
}

void TransportNetwork::constrainSlave(const Gecode::Space& lastSpace)
{
    const TransportNetwork& lastTransportNetwork = static_cast<const TransportNetwork&>(lastSpace);

    breakpointStart()
         << "constrainSlave()" << std::endl
         << "Last state: " << std::endl
         << "    # flaws: "<< lastTransportNetwork.mMinCostFlowFlaws.size() << std::endl
         << "    # resolution options: " << lastTransportNetwork.mFlawResolution.remainingDraws().size() << std::endl;
    breakpointEnd();

    //rel(*this, cost(), Gecode::IRT_LE, lastTransportNetwork.cost().val());

    // Approach 1: BFS Style
    // Slaves have to provide optimal (zero flaw solutions)
    // This has the effect that iteration of the slave either returns a
    // perfect solution or none -- yet it would be beneficial to improve any
    // 'best slave'
    rel(*this, mNumberOfFlaws, Gecode::IRT_EQ, 0);

    // Approach 2: DFS Style
    // improve any existing solution
    rel(*this, mNumberOfFlaws, Gecode::IRT_LE, lastTransportNetwork.mNumberOfFlaws);

    // TODO:
    // check on the number of flows and the cost of the solution in order to
    // find an improved solution
}

// Gecode 9.4.4
// The default slave() function does nothing and returns true , indicating that
// the search in the slave space is going to be complete. This means that if the
// search in the slave space finishes exhaustively, the meta search will also
// finish. Returning false instead would indicate that the slave search is
// incomplete, for example if it only explores a limited neighborhood of the
// previous solution
bool TransportNetwork::slave(const Gecode::MetaInfo& mi)
{
    if(!mUseMasterSlave)
    {
        // using default implementation of slave, i.e. search is complete
        return true;
    } else {
        mpMission->getLogger()->incrementSessionId();
    }

    breakpointStart()
        << "slave() with restarts: " << mi.restart() << std::endl;
    breakpointEnd();

    if(mi.type() == Gecode::MetaInfo::RESTART)
    {
        if(!mi.last())
        {
            // the previous call with initialize the selected draw
            breakpointStart()
                    << "No last space available, thus slave search is complete" << std::endl;
            breakpointEnd();

            // slave should only expand an existing solution,
            // but search is not complete at this stage
            return false;
        }

        // Last space the slave will improve upon
        // this should be consistent with the master of this slave
        const TransportNetwork& lastSpace = static_cast<const TransportNetwork&>(*mi.last());
        if(!lastSpace.mFlawResolution.next(true))
        {
            // the previous call with initialize the selected draw
            breakpointStart()
                << "Flaw resolution: options are exhausted slave search is complete" << std::endl;
            breakpointEnd();

            // Options are exhausted Search is complete
            mpMission->getLogger()->incrementSessionId();
            return false;
        } else {
            std::cout << "Calling next with last space" << std::endl;
            next(lastSpace, mi);
            return false;
        }
    }
    return false;
}

TransportNetwork::Solution TransportNetwork::getSolution() const
{
    Solution solution;
    try {
        solution.mModelDistribution = getModelDistribution();
        solution.mRoleDistribution = getRoleDistribution();
        solution.mTimelines = getTimelines();
        solution.mLocations = mLocations;
        solution.mTimepoints = mTimepoints;
        solution.mMinCostFlowSolution = mMinCostFlowSolution;
        solution.mSolutionAnalysis = mSolutionAnalysis;
    } catch(std::exception& e)
    {
        LOG_WARN_S << e.what();
    }
    return solution;
}

void TransportNetwork::saveSolution(const Solution& solution, const Mission::Ptr& mission)
{
    std::string filename;
    int i = mission->getLogger()->getSessionId();
    try {
        std::stringstream ss;
        ss << "transport-network-solution-" << i << ".dot";
        filename = mission->getLogger()->filename(ss.str());
        graph_analysis::io::GraphIO::write(filename, solution.getMinCostFlowSolution().getGraph());
    } catch(const std::exception& e)
    {
        LOG_WARN_S << "Saving file " << filename << " failed: -- " << e.what();
    }

    try {
        std::stringstream ss;
        ss << "transport-network-solution-" << i << ".gexf";
        filename = mission->getLogger()->filename(ss.str());
        solution.getMinCostFlowSolution().save(filename, "gexf");
    } catch(const std::exception& e)
    {
        LOG_WARN_S << "Saving file " << filename << " failed: -- " << e.what();
    }

    try {
        solution.getSolutionAnalysis().save();
    } catch(const std::exception& e)
    {
        LOG_WARN_S << "Saving solution analysis failed: -- " << e.what();
    }
}

TransportNetwork::ModelDistribution TransportNetwork::getModelDistribution() const
{
    ModelDistribution solution;
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, mModelPool.size(), mResourceRequirements.size());

    // Check if resource requirements holds
    for(size_t i = 0; i < mResourceRequirements.size(); ++i)
    {
        moreorg::ModelPool modelPool;
        for(size_t mi = 0; mi < mpMission->getAvailableResources().size(); ++mi)
        {
            Gecode::IntVar var = resourceDistribution(mi, i);
            if(!var.assigned())
            {
                throw std::runtime_error("templ::solvers::csp::TransportNetwork::getSolution: value has not been assigned");
            }

            Gecode::IntVarValues v( var );

            modelPool[ mAvailableModels[mi] ] = v.val();
        }

        solution[ mResourceRequirements[i] ] = modelPool;
    }
    return solution;
}

TransportNetwork::RoleDistribution TransportNetwork::getRoleDistribution() const
{
    RoleDistribution solution;

    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    // Check if resource requirements holds
    for(size_t i = 0; i < mResourceRequirements.size(); ++i)
    {
        Role::List roles;
        for(size_t r = 0; r < mRoles.size(); ++r)
        {
            Gecode::IntVar var = roleDistribution(r, i);
            if(!var.assigned())
            {
                throw std::runtime_error("templ::solvers::csp::RoleDistribution::getSolution: value has not been assigned for role: '" + mRoles[r].toString() + "'");
            }

            Gecode::IntVarValues v( var );

            if( v.val() == 1 )
            {
                roles.push_back( mRoles[r] );
            }
        }

        solution[ mResourceRequirements[i] ] = roles;
    }

    return solution;
}

std::map<Role, csp::RoleTimeline> TransportNetwork::getTimelines() const
{
    std::map<Role, csp::RoleTimeline> roleTimelines;
    for(size_t i = 0; i < mActiveRoleList.size(); ++i)
    {
        const Role& role = mActiveRoleList[i];
        LOG_INFO_S << "Active role: " << i << " of " << mActiveRoleList.size() << " " << mActiveRoleList[i].toString() << std::endl
            << Formatter::toString(mTimelines[i],
                    mLocations,
                    mTimepoints)
            << std::endl;

        bool doThrow = false;
        SpaceTime::Timeline timeline = TypeConversion::toTimeline(mTimelines[i],
                mLocations,
                mTimepoints,
                doThrow);

        csp::RoleTimeline roleTimeline(role, mAsk);
        roleTimeline.setTimeline(timeline);

        roleTimelines[role] = roleTimeline;

    }
    return roleTimelines;
}

TransportNetwork::TransportNetwork()
    : Gecode::Space()
    , Solver(Solver::CSP_TRANSPORT_NETWORK)
    , mAsk()
{}

TransportNetwork::TransportNetwork(const templ::Mission::Ptr& mission, const qxcfg::Configuration& configuration)
    : Gecode::Space()
    , Solver(Solver::CSP_TRANSPORT_NETWORK)
    , mpMission(mission)
    , mModelPool(mpMission->getAvailableResources())
    , mAsk(mpMission->getOrganizationModel(),
            mpMission->getAvailableResources(), true,
            1000*configuration.getValueAs<double>("TransportNetwork/search/options/connectivity/timeout_in_s",20),
            configuration.getValue("TransportNetwork/search/options/connectivity/interface-type",moreorg::vocabulary::OM::ElectroMechanicalInterface().toString())
          )
    , mResources(mpMission->getRequestedResources())
    , mIntervals(mpMission->getTimeIntervals())
    , mTimepoints(mpMission->getUnorderedTimepoints())
    , mLocations(mpMission->getLocations())
    , mResourceRequirements()
    , mQualitativeTimepoints(*this, mpMission->getQualitativeTemporalConstraintNetwork()->getTimepoints().size(), 0, mpMission->getQualitativeTemporalConstraintNetwork()->getTimepoints().size()-1)
    , mAvailableModels(mpMission->getModels())
    , mModelUsage()
    , mRoleUsage()
    , mRoles(mission->getRoles())
    , mCost(*this,0, Gecode::Int::Limits::max)
    , mNumberOfFlaws(*this,0, Gecode::Int::Limits::max)
    , mConfiguration(configuration)
    , mUseMasterSlave(false)
    , mpCurrentMaster(NULL)
   // , mCapacities(*this, (mLocations.size()+1)*(mLocations.size()+1)*mTimepoints.size()*mTimepoints.size(), 0, Gecode::Int::Limits::max)
{
    // FIXME: make sure we use the the same configuration of the ask object
    mpMission->setOrganizationModelAsk(mAsk);

    assert( mpMission->getOrganizationModel() );
    assert(!mIntervals.empty());
    LOG_INFO_S << "TransportNetwork CSP Problem Construction" << std::endl
    << "    requested resources: " << mResources << std::endl
    << "    intervals: " << mIntervals.size() << std::endl
    << "    # requirements: " << mResourceRequirements.size() << std::endl;

    initializeTemporalConstraintNetwork();
}

void TransportNetwork::initializeTemporalConstraintNetwork()
{
    // Allow branching of temporal constraint network
    // Initialize constraint network after mQualitativeTimepoints has been
    // properly constructed -- otherwise we will trigger segfaults
    mTemporalConstraintNetwork = TemporalConstraintNetworkBase(*mpMission->getQualitativeTemporalConstraintNetwork(),*this, mQualitativeTimepoints);

    bool nooverlap = mConfiguration.getValueAs<bool>("TransportNetwork/intervals-nooverlap",false);
    if(nooverlap)
    {
        LOG_WARN_S << "Configuration: no interval overlaps are allowed";
        mTemporalConstraintNetwork.addNoOverlap(mIntervals, *this, mQualitativeTimepoints);
    }
    // making sure we get a fully assigned temporal constraint network, i.e.
    // one without gaps before we proceed
    Gecode::Rnd temporalNetworkRnd;
    temporalNetworkRnd.hw();
    Gecode::branch(*this, mQualitativeTimepoints, Gecode::INT_VAR_RND(temporalNetworkRnd), Gecode::INT_VAL_MIN());
    Gecode::branch(*this, &TransportNetwork::doPostTemporalConstraints);
}


void TransportNetwork::initializeMinMaxConstraints()
{
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());

    // TODO: change to using MissionConstraintManager
    // For debugging purposes
    ConstraintMatrix constraintMatrix(mAvailableModels);
    using namespace solvers::temporal;
    std::vector<FluentTimeResource>::const_iterator fit = mResourceRequirements.begin();
    for(; fit != mResourceRequirements.end(); ++fit)
    {
        const FluentTimeResource& fts = *fit;
        // row: index of requirement
        // col: index of model type
        size_t requirementIndex = fit - mResourceRequirements.begin();
        for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
        {
            Gecode::IntVar v = resourceDistribution(mi, requirementIndex);
            uint32_t minCardinality = 0;
            const owlapi::model::IRI& model = mAvailableModels[mi];
            {
                // default min requirement is 0 for a model
                /// Consider resource cardinality constraint
                /// Check what is set for the given model
                moreorg::ModelPool::const_iterator cardinalityIt =
                    fts.getMinCardinalities().find(model);
                if(cardinalityIt != fts.getMinCardinalities().end())
                {
                    minCardinality = cardinalityIt->second;
                }
                constraintMatrix.setMin(requirementIndex, mi, minCardinality);
                rel(*this, v, Gecode::IRT_GQ, minCardinality);
            }

            uint32_t maxCardinality = mModelPool[ model ];
            // setting the upper bound for this model and this service
            // based on what the model pool can provide
            constraintMatrix.setMax(requirementIndex, mi, maxCardinality);
            rel(*this, v, Gecode::IRT_LQ, maxCardinality);

            LOG_DEBUG_S << "requirement: " << requirementIndex
                << ", model: " << model
                << " IRT_GQ " << minCardinality << ", IRT_LQ: " << maxCardinality;
        }

        // there can be no empty assignment for resource requirement
        rel(*this, sum( resourceDistribution.row(requirementIndex) ) > 0);
        if(this->failed())
        {
            LOG_WARN_S << "Encountered an empty assignment for a resource requirement" << std::endl
                << "requirement index: " << requirementIndex << std::endl
                << resourceDistribution.row(requirementIndex) << std::endl
                << fts.toString(4);
            return;
        }
    }

    std::vector<std::string> rowNames =
        FluentTimeResource::toQualificationStringList(mResourceRequirements.begin(),
            mResourceRequirements.end());
    LOG_INFO_S << constraintMatrix.toString(rowNames);

    breakpointStart()
        << "InitializeMinMax: final constraint matrix: " << constraintMatrix.toString(rowNames) << std::endl;
    breakpointEnd();
}

void TransportNetwork::addExtensionalConstraints()
{
    size_t availableResourceCount = mpMission->getAvailableResources().size();
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage,
            /*width --> col*/ availableResourceCount,
            /*height --> row*/ mResourceRequirements.size());

   size_t requirementIndex = 0;
   for(const FluentTimeResource& ftr: mResourceRequirements)
   {
        // Prepare the extensional constraints, i.e. specifying the allowed
        // combinations for each requirement
        moreorg::ModelPool::Set allowedCombinations = ftr.getDomain();
        if(allowedCombinations.empty())
        {
            LOG_WARN_S << "No allowed combinations available with the given constraints: failing this space";
            this->fail();
            return;
        }
        LOG_INFO_S << "Adding extensional constraint:\n" << ftr.toString(4);

        // A tuple set is a fully expanded vector describing the cardinality for
        // all available resources
        Gecode::TupleSet tupleSet(availableResourceCount);
        appendToTupleSet(tupleSet, allowedCombinations);
        tupleSet.finalize();
        extensional(*this, resourceDistribution.row(requirementIndex), tupleSet);
        if(this->failed())
        {
            LOG_WARN_S  << "Adding extensional constraint lead to failed space"
                << ftr.toString(4);
            return;
        }
        ++requirementIndex;
   }
}

void TransportNetwork::setUpperBoundForConcurrentRequirements()
{
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());

    // - identify overlapping fts, limit resources for these
    std::vector< std::vector<FluentTimeResource> > concurrentRequirements;
    bool nooverlap = mConfiguration.getValueAs<bool>("TransportNetwork/intervals-nooverlap",false);

    if(nooverlap)
    {
        for(const FluentTimeResource& ftr : mResourceRequirements)
        {
            concurrentRequirements.push_back( { ftr } );
        }
    } else {
        // Make sure the correct constraints network is used for comparison
        temporal::point_algebra::TimePointComparator tpc(mpQualitativeTemporalConstraintNetwork);
        // Make sure the assignments are within resource bounds for concurrent requirements
        concurrentRequirements = FluentTimeResource::getMutualExclusive(mResourceRequirements, tpc);
    }

    for(const std::vector<FluentTimeResource>& concurrentFluents :
            concurrentRequirements)
    {
        for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
        {
            const owlapi::model::IRI& model = mAvailableModels[mi];
            Gecode::IntVarArgs args;

            std::vector<FluentTimeResource>::const_iterator fit = concurrentFluents.begin();
            for(; fit != concurrentFluents.end(); ++fit)
            {
                size_t fluentIdx = FluentTimeResource::getIndex(mResourceRequirements, *fit);
                Gecode::IntVar v = resourceDistribution(mi,fluentIdx);
                args << v;
            }

            uint32_t maxCardinality = mModelPool[ model ];
            LOG_DEBUG_S << "Add general resource usage constraint: " << std::endl
                << "     " << mAvailableModels[mi].toString() << "# <= " << maxCardinality;
            rel(*this, sum(args) <= maxCardinality);
        }
    }
}

void TransportNetwork::initializeRoleDistributionConstraints()
{
    bool forceMinimumRoleUsage = mConfiguration.getValueAs<bool>("TransportNetwork/search/options/role-usage/force-min",false);
    int mobileRoleUsageBoundOffset = mConfiguration.getValueAs<int>("TransportNetwork/search/options/role-usage/mobile/bound-offset",0);
    bool mobileBoundedRoleUsage = mConfiguration.getValueAs<bool>("TransportNetwork/search/options/role-usage/mobile/bounded",false);
    int immobileRoleUsageBoundOffset = mConfiguration.getValueAs<int>("TransportNetwork/search/options/role-usage/immobile/bound-offset",0);
    bool immobileBoundedRoleUsage = mConfiguration.getValueAs<bool>("TransportNetwork/search/options/role-usage/immobile/bounded",false);

    // Role distribution
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());
    {
        Gecode::IntVarArgs mobileModelBounds;
        Gecode::IntVarArgs immobileModelBounds;

        Gecode::IntVarArgs mobileModelMaxOffsets;
        Gecode::IntVarArgs immobileModelMaxOffsets;

        // Sum of all models' instances (role) has to correspond to the model count
        for(size_t modelIndex = 0; modelIndex < mAvailableModels.size(); ++modelIndex)
        {
            const owlapi::model::IRI& model = mAvailableModels[modelIndex];

            Gecode::IntVar mobileModelBound(*this,0, mobileRoleUsageBoundOffset);
            mobileModelBounds << mobileModelBound;
            Gecode::IntVar immobileModelBound(*this,0, immobileRoleUsageBoundOffset);
            immobileModelBounds << immobileModelBound;

            using namespace moreorg::facades;
            Robot robot = Robot::getInstance(model, mAsk);
            bool isMobile = robot.isMobile();
            uint32_t maxCardinality = mModelPool[ model ];

            // Enforce bound per requirement
            for(uint32_t requirementIndex = 0; requirementIndex < mResourceRequirements.size(); ++requirementIndex)
            {
                Gecode::IntVar modelCount = resourceDistribution(modelIndex,requirementIndex);
                Gecode::IntVarArgs args;
                for(uint32_t roleIndex = 0; roleIndex < mRoles.size(); ++roleIndex)
                {
                    if(isRoleForModel(roleIndex, modelIndex))
                    {
                        Gecode::IntVar roleActivation = roleDistribution(roleIndex, requirementIndex);
                        args << roleActivation;
                    }
                }
                Gecode::IntVar z(*this, 0, maxCardinality);
                rel(*this, z == sum(args) );

                // The following constraint allow to relax the least commitment
                // and allows to relax the number of immobile / mobile agent
                // assigned
                if(forceMinimumRoleUsage)
                {
                    // This tries to solve the problem with the fewest instances
                    // possible
                    rel(*this, z == modelCount);
                } else
                {
                    rel(*this, z >= modelCount);
                    // This tries to bound the problem using the number of
                    // available instances
                    LOG_DEBUG_S << "Constraint general role usage: " << std::endl
                        << "     " << model.toString() << "# <= " << maxCardinality;
                    // Limit mobile / immobile systems
                    if(isMobile)
                    {

                        if(mobileBoundedRoleUsage)
                        {
                            // This tries to bound the problem using the number of
                            // available instances
                            rel(*this, z <= (modelCount + mobileModelBound) );
                            LOG_DEBUG_S << "Constraint mobile role usage: " << std::endl
                                << "     " << model.toString() << "#: " << modelCount << "<= x <= " << modelCount << "+ " << mobileRoleUsageBoundOffset;
                        } else {
                            rel(*this, z == modelCount);
                        }
                    } else
                    { // not mobile
                        if(immobileBoundedRoleUsage)
                        {
                            // This tries to bound the problem using the number of
                            // available instances
                            rel(*this, z <= (modelCount + immobileModelBound) );
                            LOG_DEBUG_S << "Constraint immobile role usage: " << std::endl
                                << "     " << model.toString() << "#: " << modelCount << "<= x <= " << modelCount << "+ " << immobileRoleUsageBoundOffset;
                        } else {
                            rel(*this, z == modelCount);
                        }
                    }
                }
            }

            Gecode::IntVar mobileModelOffsetMax(*this,0, mobileRoleUsageBoundOffset);
            Gecode::max(*this, mobileModelBounds, mobileModelOffsetMax);
            mobileModelMaxOffsets << mobileModelOffsetMax;

            Gecode::IntVar immobileModelOffsetMax(*this,0, immobileRoleUsageBoundOffset);
            Gecode::max(*this, immobileModelBounds, immobileModelOffsetMax);
            immobileModelMaxOffsets << immobileModelOffsetMax;
        }

        if(!forceMinimumRoleUsage)
        {
            // Making sure the total offset of model instances does not exceed the
            // given bound, i.e. if model bound is 2, that means only two
            // additional model instances will be available
            if(mobileBoundedRoleUsage)
            {
                rel(*this, sum(mobileModelMaxOffsets) <= mobileRoleUsageBoundOffset);
            }
            if(immobileBoundedRoleUsage)
            {
                rel(*this, sum(immobileModelMaxOffsets) <= immobileRoleUsageBoundOffset);
            }
        }
    }
}

void TransportNetwork::applyMissionConstraints()
{
    for(const Constraint::Ptr& constraint : mpMission->getConstraints())
    {
        MissionConstraintManager::apply(constraint, *this);
    }
}

void TransportNetwork::applyExtraConstraints()
{
    for(const Constraint::Ptr& constraint : mConstraints)
    {
        MissionConstraintManager::apply(constraint, *this);
    }
}

void TransportNetwork::applyAccessConstraints(ListOfAdjacencyLists& timelines,
        size_t numberOfTimepoints,
        size_t numberOfLocations,
        const Role::List& roles)
{
    // Location access contraints -- currently restricted to a single model
    using namespace moreorg;
    typedef std::map<symbols::constants::Location::Ptr, std::pair<ModelPool, ModelPool> >
        LocationConstraints;
    LocationConstraints locationMinMax;

    Constraint::PtrList constraints = mpMission->getConstraints();
    constraints.insert(constraints.begin(),mConstraints.begin(), mConstraints.end());

    for(const Constraint::Ptr& constraint : constraints)
    {
        using namespace templ::constraints;
        if(constraint->getCategory() == Constraint::MODEL)
        {
            ModelConstraint::Ptr modelConstraint =
                dynamic_pointer_cast<ModelConstraint>(constraint);

            ModelConstraint::Type modelConstraintType =
                modelConstraint->getModelConstraintType();

            if(modelConstraintType != ModelConstraint::MIN_ACCESS
                    && modelConstraintType != ModelConstraint::MAX_ACCESS)
            {
                continue;
            }

            bool isGeneralAccessConstraint = false;
            std::pair<ModelPool, ModelPool> minMax;
            symbols::constants::Location::Ptr affectedLocation;

            for(const SpaceTime::SpaceIntervalTuple& t :
                    modelConstraint->getSpaceIntervalTuples())
            {
                if(SpaceTime::isFullMissionInterval(t.second()))
                {
                    isGeneralAccessConstraint = true;
                    affectedLocation = t.first();
                    LocationConstraints::const_iterator cit =
                        locationMinMax.find(affectedLocation);
                    if(cit != locationMinMax.end())
                    {
                        minMax = cit->second;
                        break;
                    }
                }
            }
            if(!isGeneralAccessConstraint)
            {
                // this is not covers
                continue;
            }

            using namespace templ::constraints;
            const owlapi::model::IRI& model = modelConstraint->getModel();

            // min and max entries are maintained in parallel
            if(minMax.first.count(model) == 0)
            {
                minMax.first[model] = 0;
                minMax.second[model] = std::numeric_limits<size_t>::max();
            }
            size_t& min = minMax.first[model];
            size_t& max = minMax.second[model];

            switch(modelConstraint->getModelConstraintType())
            {
                case ModelConstraint::MIN_ACCESS:
                    min = std::max(min, static_cast<size_t>(modelConstraint->getValue()));
                    break;
                case ModelConstraint::MAX_ACCESS:
                    max = std::min(max, static_cast<size_t>(modelConstraint->getValue()));
                    break;
                default:
                    continue;
            }
            // update the set of constraints
            locationMinMax[affectedLocation] = minMax;
        } // end for constraints
    }

    for(const LocationConstraints::value_type& locationConstraint : locationMinMax)
    {
        ListOfAdjacencyLists selectedTimelines;
        const symbols::constants::Location::Ptr& location = locationConstraint.first;
        const ModelPool& minPool = locationConstraint.second.first;
        const ModelPool& maxPool = locationConstraint.second.second;

        // handle location constraints for each model
        for(const ModelPool::value_type& v : minPool)
        {
            // Get all models that are a subclass of the given
            const owlapi::model::IRI& model = v.first;
            for(size_t i = 0; i < roles.size(); ++i)
            {
                if(mAsk.ontology().isSubClassOf( roles[i].getModel(), model ))
                {
                    selectedTimelines.push_back(timelines[i]);
                }
            }

            symbols::constants::Location::PtrList::const_iterator cit =
                std::find(mLocations.begin(), mLocations.end(), location);
            if(cit == mLocations.end())
            {
                throw std::runtime_error("templ::solvers::csp::TransportNetwork::applyAccessConstraints:"
                            " failed to find location " + location->toString());
            }
            size_t locationIdx = cit - mLocations.begin();
            size_t min = minPool.at(model);
            size_t max = maxPool.at(model);

            propagators::restrictInEdges(*this,
                    selectedTimelines,
                    numberOfTimepoints,
                    numberOfLocations,
                    locationIdx,
                    min,
                    max,
                    location->toString() + "-" + model.toString());
        }
    }
}

void TransportNetwork::enforceUnaryResourceUsage()
{
    // Role distribution
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    // Set of available models: mModelPool
    // Make sure the assignments are within resource bounds for concurrent requirements
    temporal::point_algebra::TimePointComparator tpc(mpQualitativeTemporalConstraintNetwork);
    std::vector< std::vector<FluentTimeResource> > concurrentRequirements =
        FluentTimeResource::getMutualExclusive(mResourceRequirements, tpc);

    for(const FluentTimeResource::List& concurrentFluents : concurrentRequirements)
    {
        for(size_t roleIndex = 0; roleIndex < mRoles.size(); ++roleIndex)
        {
            Gecode::IntVarArgs args;
            for(const FluentTimeResource& fts : concurrentFluents)
            {
                size_t row = FluentTimeResource::getIndex(mResourceRequirements, fts);
                Gecode::IntVar v = roleDistribution(roleIndex, row);
                args << v;
            }
            // there can only be one role active
            rel(*this, sum(args) <= 1);
        }
    }
}

Gecode::Symmetries TransportNetwork::identifySymmetries()
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    Gecode::Symmetries symmetries;
    // define interchangeable columns for roles of the same model type
    for(const owlapi::model::IRI& currentModel : mAvailableModels)
    {
        LOG_INFO_S << "Starting symmetry column for model: " << currentModel.toString();
        Gecode::IntVarArgs sameModelColumns;
        for(int c = 0; c < roleDistribution.width(); ++c)
        {
            if( mRoles[c].getModel() == currentModel)
            {
                LOG_DEBUG_S << "Adding column of " << mRoles[c].toString() << " for symmetry";
                sameModelColumns << roleDistribution.col(c);
            }
        }
        symmetries << VariableSequenceSymmetry(sameModelColumns, roleDistribution.height());
    }
    return symmetries;
}

TransportNetwork::TransportNetwork(TransportNetwork& other)
    : Gecode::Space(other)
    , mpMission(other.mpMission)
    , mModelPool(other.mModelPool)
    , mAsk(other.mAsk)
    , mServices(other.mServices)
    , mResources(other.mResources)
    , mIntervals(other.mIntervals)
    , mTimepoints(other.mTimepoints)
    , mLocations(other.mLocations)
    , mResourceRequirements(other.mResourceRequirements)
    , mTemporalConstraintNetwork(other.mTemporalConstraintNetwork)
    , mpQualitativeTemporalConstraintNetwork(other.mpQualitativeTemporalConstraintNetwork)
    , mAvailableModels(other.mAvailableModels)
    , mRoles(other.mRoles)
    , mActiveRoles(other.mActiveRoles)
    , mActiveRoleList(other.mActiveRoleList)
    , mMinRequiredTimelines(other.mMinRequiredTimelines)
    , mSupplyDemand(other.mSupplyDemand)
    , mMinCostFlowSolution(other.mMinCostFlowSolution)
    , mMinCostFlowFlaws(other.mMinCostFlowFlaws)
    , mFlawResolution(other.mFlawResolution)
    , mConfiguration(other.mConfiguration)
    , mUseMasterSlave(other.mUseMasterSlave)
    , mpCurrentMaster(other.mpCurrentMaster)
    , mSolutionAnalysis(other.mSolutionAnalysis)
{
    breakpointStart()
        << "Space " << std::endl
        << "    size: " << sizeof(TransportNetwork) << std::endl
        << "Mission: " << std::endl
        << "    use count: " << mpMission.use_count() << std::endl
        << "QualitativeTemporalConstraintNetwork" << std::endl
        << "    use count: " << mpQualitativeTemporalConstraintNetwork.use_count() << std::endl;
    breakpointEnd();

    assert( mpMission->getOrganizationModel() );
    assert(!mIntervals.empty());
    mModelUsage.update(*this, other.mModelUsage);
    mRoleUsage.update(*this, other.mRoleUsage);
    mCost.update(*this, other.mCost);
    mNumberOfFlaws.update(*this, other.mNumberOfFlaws);
    mQualitativeTimepoints.update(*this, other.mQualitativeTimepoints);

    for(size_t i = 0; i < other.mTimelines.size(); ++i)
    {
        AdjacencyList array;
        mTimelines.push_back(array);
        mTimelines[i].update(*this, other.mTimelines[i]);
    }

    //mCapacities.update(*this, other.mCapacities);
}

Gecode::Space* TransportNetwork::copy()
{
    return new TransportNetwork(*this);
}

std::vector<TransportNetwork::Solution> TransportNetwork::solve(const templ::Mission::Ptr& mission, uint32_t minNumberOfSolutions, const qxcfg::Configuration& configuration)
{
    SolutionList solutions;
    mission->validateForPlanning();

    assert(mission->getOrganizationModel());
    assert(!mission->getTimeIntervals().empty());

    /// Allow to log the final results into a csv file
    CSVLogger csvLogger({"session",
            "alpha",
            "beta",
            "sigma",
            "efficacy",
            "efficiency",
            "safety",
            "timehorizon",
            "travel-distance",
            "reconfiguration-cost",
            "overall-runtime",
            "solution-runtime",
            "solution-runtime-mean",
            "solution-runtime-stdev",
            "solution-found",
            "solution-stopped",
            "propagate",
            "fail",
            "node",
            "depth",
            "restart",
            "nogood",
            "flaws",
            "cost"});

    std::string baseDir = configuration.getValue("TransportNetwork/logging/basedir","/tmp");
    mission->getLogger()->setBaseDirectory(baseDir);

    if( configuration.getValueAs<bool>("TransportNetwork/use-transfer-location"))
    {
        mission->enableTransferLocation();
    }

    /// Check if interactive mode should be used during the solution process
    TransportNetwork::msInteractive = configuration.getValueAs<bool>("TransportNetwork/search/interactive",false);

    TransportNetwork* distribution = new TransportNetwork(mission, configuration);
    distribution->mUseMasterSlave = configuration.getValueAs<bool>("TransportNetwork/search/options/master-slave",false);

    // Search options: Gecode 9.3.1
    // threads (double) number of parallel threads to use
    // c_d (unsigned int) commit recomputation distance
    // a_d                adaptive recomputation distance
    // clone (bool)  whether engine uses a clone when created
    // d_l           discrepancy limit when using LDS
    // nogoods_limit depth limit for no-good generation
    // assets        number of assets in a portfolio
    // share_rbs     (bool) whether AFC is shared between restarts
    // share_pbs     (bool) whether AFC is shared between assets
    // stop                 Stop object   (NULL if none)
    // cutoff               cutoff object (NULL if none)
    int threads = distribution->mConfiguration.getValueAs<int>("TransportNetwork/search/options/threads",1);

    int cutoff = distribution->mConfiguration.getValueAs<int>("TransportNetwork/search/options/cutoff",10);
    int nogoods_limit = distribution->mConfiguration.getValueAs<int>("TransportNetwork/search/options/nogoods_limit",128);
    int epochTimeoutInS = distribution->mConfiguration.getValueAs<int>("TransportNetwork/search/options/epoch_timeout_in_s",180000);
    int abortTimeoutInS = distribution->mConfiguration.getValueAs<int>("TransportNetwork/search/options/total_timeout_in_s",600000);

    Gecode::Search::Options options;
    options.threads = threads;
    // p 172 "the value of nogoods_limit described to which depth limit
    // no-goods should be extracted from the path of the search tree
    // maintained by the search engine
    options.nogoods_limit = nogoods_limit;
    // recomputation distance
    options.c_d = distribution->mConfiguration.getValueAs<int>("TransportNetwork/search/options/computation_distance", options.c_d);
    // adaptive recomputation distance
    options.a_d = distribution->mConfiguration.getValueAs<int>("TransportNetwork/search/options/adaptive_computation_distance", options.a_d);
    // default failure cutoff
    // options.fail

    base::Time allStart = base::Time::now();
    bool stop = false;
    int numberOfEpochs = 0;
    while(!stop)
    {
        ++numberOfEpochs;
        //Cutoff::geometric: s*b^i, for i = 0,1,2,3,4
        // when the corresponding number of failure has been reached
        // restart and continue
        options.cutoff = Gecode::Search::Cutoff::geometric(cutoff,2);
        options.stop = Gecode::Search::Stop::time(epochTimeoutInS*1000.0);
        Gecode::RBS< TransportNetwork, Gecode::DFS > searchEngine(distribution, options);

        //Gecode::TemplRBS< TransportNetwork, Gecode::DFS > searchEngine(distribution, options);

        TransportNetwork* best = NULL;
        size_t solutionCount = 0;
        int i = 0;
        base::Time start = base::Time::now();
        base::Time allElapsed;
        base::Time elapsed;
        numeric::Stats<double> stats;
        while(TransportNetwork* current = searchEngine.next())
        {
            allElapsed = (base::Time::now() - allStart);
            elapsed = (base::Time::now() - start);
            stats.update(elapsed.toSeconds());
            delete best;
            best = current;

            using namespace moreorg;

            LOG_INFO_S << "#" << i << "/" << minNumberOfSolutions << " solution found:" << current->toString();
            std::cout << "Solution found:" << std::endl;
            std::cout << "    # session id " << current->mpMission->getLogger()->getSessionId() << std::endl;
            std::cout << "    # flaws: " << current->mNumberOfFlaws.val() << std::endl;
            std::cout << "    # cost: " << current->mCost.val() << std::endl;
            std::cout << std::endl;

            csvLogger.addToRow(current->mpMission->getLogger()->getSessionId(),"session");
            csvLogger.addToRow(current->mSolutionAnalysis.getAlpha(), "alpha");
            csvLogger.addToRow(current->mSolutionAnalysis.getBeta(), "beta");
            csvLogger.addToRow(current->mSolutionAnalysis.getSigma(), "sigma");
            csvLogger.addToRow(current->mSolutionAnalysis.getEfficacy(), "efficacy");
            csvLogger.addToRow(current->mSolutionAnalysis.getEfficiency(), "efficiency");
            csvLogger.addToRow(current->mSolutionAnalysis.getSafety(), "safety");
            csvLogger.addToRow(current->mSolutionAnalysis.getTimeHorizon(), "timehorizon");
            csvLogger.addToRow(current->mSolutionAnalysis.getTravelledDistance(),"travel-distance");
            csvLogger.addToRow(current->mSolutionAnalysis.getReconfigurationCost(),"reconfiguration-cost");
            csvLogger.addToRow(allElapsed.toSeconds(), "overall-runtime");
            csvLogger.addToRow(elapsed.toSeconds(), "solution-runtime");
            csvLogger.addToRow(stats.mean(), "solution-runtime-mean");
            csvLogger.addToRow(stats.stdev(), "solution-runtime-stdev");
            csvLogger.addToRow(searchEngine.stopped(), "solution-stopped");
            csvLogger.addToRow(searchEngine.statistics().propagate, "propagate");
            csvLogger.addToRow(searchEngine.statistics().fail, "fail");
            csvLogger.addToRow(searchEngine.statistics().node, "node");
            csvLogger.addToRow(searchEngine.statistics().depth, "depth");
            csvLogger.addToRow(searchEngine.statistics().restart, "restart");
            csvLogger.addToRow(searchEngine.statistics().nogood, "nogood");
            csvLogger.addToRow(1.0, "solution-found");
            csvLogger.addToRow(best->mMinCostFlowFlaws.size(), "flaws");
            csvLogger.addToRow(best->cost().val(), "cost");
            csvLogger.commitRow();

            std::string filename =
                mission->getLogger()->getBasePath() + "search-statistics.log";
            std::cout << "Saving stats in: " << filename << std::endl;
            csvLogger.save(filename);

            Solution solution = current->getSolution();
            saveSolution(solution, mission);
            // TODO: use serialization to filesystem for later retrieval
            solutions.push_back(solution);
            ++solutionCount;

            if(minNumberOfSolutions != 0)
            {
                if(solutionCount >= minNumberOfSolutions)
                {
                    LOG_INFO_S << "Found minimum required number of solutions: " << solutions.size();
                    stop = true;
                    break;
                }
            }

            current->mpMission->getLogger()->incrementSessionId();
            start = base::Time::now();
        }

        std::cout << "Solution Search (epoch: " << numberOfEpochs << ")" << std::endl;
        std::cout << "    was stopped (e.g. timeout): ";
        if(searchEngine.stopped())
        {
            std::cout << " yes" << std::endl;
        } else {
            std::cout << " no" << std::endl;
        }
        std::cout << "    found # solutions: " << solutions.size() << std::endl;
        std::cout << "    minimum # requested: " << minNumberOfSolutions << std::endl;

        if((base::Time::now() - allStart).toSeconds() >= abortTimeoutInS)
        {
            stop = true;
        }
    } // end while all

    delete distribution;
    return solutions;
}

solvers::Session::Ptr TransportNetwork::run(const templ::Mission::Ptr& mission, uint32_t minNumberOfSolutions, const qxcfg::Configuration& configuration)
{
    Session::Ptr session = make_shared<Session>(mission);
    SolutionList solutionList = TransportNetwork::solve(mission, minNumberOfSolutions, configuration);

    solvers::Solution::List solutions;
    session->setSolutions(solutions);
    return session;
}

void TransportNetwork::addConstraint(const Constraint::Ptr& constraint, TransportNetwork& network)
{
    mConstraints.push_back(constraint);
    MissionConstraintManager::apply(constraint, network);
}

Constraint::PtrList TransportNetwork::getAssignmentsAsConstraints() const
{
    using namespace moreorg;
    Constraint::PtrList constraints;
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());


    // Min resource model constraints
    for(size_t f = 0; f < mResourceRequirements.size(); ++f)
    {
        const FluentTimeResource& ftr = mResourceRequirements[f];

        ModelPool modelPool = currentMinModelAssignment(ftr);
        for(const ModelPool::value_type& v : modelPool)
        {
            constraints::ModelConstraint::Ptr constraint = make_shared<constraints::ModelConstraint>(
                    constraints::ModelConstraint::MIN,
                    v.first,
                    MissionConstraintManager::mapToSpaceTime(ftr),
                    v.second
                    );
            constraints.push_back(constraint);
        }
    }

    for(size_t r = 0; r < mRoles.size(); ++r)
    {
        FluentTimeResource::List presentAt;
        for(size_t f = 0; f < mResourceRequirements.size(); ++f)
        {
            Gecode::IntVar var = roleDistribution(r,f);
            if(var.assigned() && var.val() == 1)
            {
                presentAt.push_back( mResourceRequirements[f] );
            }
        }

        if(!presentAt.empty())
        {
            constraints::ModelConstraint::Ptr constraint = make_shared<constraints::ModelConstraint>(
                    constraints::ModelConstraint::MIN_EQUAL,
                    mRoles[r].getModel(),
                    MissionConstraintManager::mapToSpaceTime( presentAt ),
                    1
                    );
            constraints.push_back(constraint);
        }
    }

    return constraints;

}

Mission::Ptr TransportNetwork::getAugmentedMission() const
{
    Mission::Ptr augmentedMission = make_shared<Mission>(*mpMission);
    Constraint::PtrList constraints = getAssignmentsAsConstraints();
    for(Constraint::Ptr& constraint : constraints)
    {
        augmentedMission->addConstraint(constraint);
    }
    return augmentedMission;
}

void TransportNetwork::appendToTupleSet(Gecode::TupleSet& tupleSet, const moreorg::ModelPool::Set& combinations) const
{
    std::set< std::vector<uint32_t> > csp = utils::Converter::toCSP(mpMission, combinations);
    std::set< std::vector<uint32_t> >::const_iterator cit = csp.begin();

    for(; cit != csp.end(); ++cit)
    {
        Gecode::IntArgs args;

        const std::vector<uint32_t>& tuple = *cit;
        std::vector<uint32_t>::const_iterator tit = tuple.begin();
        for(; tit != tuple.end(); ++tit)
        {
            args << *tit;
        }
        LOG_DEBUG_S << "TupleSet: intargs: " << args;

        tupleSet.add( args );
    }
}

size_t TransportNetwork::getResourceModelIndex(const owlapi::model::IRI& model) const
{
    owlapi::model::IRIList::const_iterator cit = std::find(mAvailableModels.begin(), mAvailableModels.end(), model);
    if(cit != mAvailableModels.end())
    {
        int index = cit - mAvailableModels.begin();
        assert(index >= 0);
        return (size_t) index;
    }

    throw std::runtime_error("templ::solvers::csp::TransportNetwork::getResourceModelIndex: could not find model index for '" + model.toString() + "'");
}

const owlapi::model::IRI& TransportNetwork::getResourceModelFromIndex(size_t index) const
{
    if(index < mAvailableModels.size())
    {
        return mAvailableModels.at(index);
    }
    throw std::invalid_argument("templ::solvers::csp::TransportNetwork::getResourceModelIndex: index is out of bounds");
}

size_t TransportNetwork::getResourceModelMaxCardinality(size_t index) const
{
    moreorg::ModelPool::const_iterator cit = mModelPool.find(getResourceModelFromIndex(index));
    if(cit != mModelPool.end())
    {
        return cit->second;
    }
    throw std::invalid_argument("templ::solvers::csp::TransportNetwork::getResourceModelMaxCardinality: model not found");
}

bool TransportNetwork::isRoleForModel(uint32_t roleIndex, uint32_t modelIndex) const
{
    return mRoles.at(roleIndex).getModel() == mAvailableModels.at(modelIndex);
}

std::vector<uint32_t> TransportNetwork::computeActiveRoles() const
{
    std::vector<uint32_t> activeRoles;
    // Identify active roles
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());
    for(size_t r = 0; r < mRoles.size(); ++r)
    {
        size_t requirementCount = 0;
        for(size_t i = 0; i < mResourceRequirements.size(); ++i)
        {
            Gecode::IntVar var = roleDistribution(r,i);
            if(!var.assigned())
            {
                throw std::runtime_error("templ::solvers::csp::TransportNetwork::postRoleAssignments: value has not been assigned for role: '" + mRoles[r].toString() + "'");
            }
            Gecode::IntVarValues v(var);
            if(v.val() == 1)
            {
                ++requirementCount;
            }

            // Only if the resource is required at more than its current start
            // position, we consider it to be active
            if(requirementCount > 1)
            {
                activeRoles.push_back(r);
                break;
            }
        }
    }
    return activeRoles;
}

moreorg::ModelPool TransportNetwork::currentMinModelAssignment(const FluentTimeResource& ftr) const
{
    moreorg::ModelPool modelPool;
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    size_t ftrIdx = FluentTimeResource::getIndex(mResourceRequirements, ftr);

    for(size_t r = 0; r < mRoles.size(); ++r)
    {
        Gecode::IntVar var = roleDistribution(r,ftrIdx);
        if(var.assigned() && var.val() == 1)
        {
            modelPool[ mRoles[r].getModel() ] += 1;
        }
    }
    return modelPool;
}

void TransportNetwork::doPostTemporalConstraints(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).postTemporalConstraints();
}

void TransportNetwork::postTemporalConstraints()
{
    (void) status();
    // Update temporal constraint network after the solution has been computed
    mpQualitativeTemporalConstraintNetwork = mTemporalConstraintNetwork.translate(mQualitativeTimepoints);
    temporal::point_algebra::TimePointComparator tcp(mpQualitativeTemporalConstraintNetwork);

    // Sort the timepoints according
    TemporalConstraintNetworkBase::sort(*mpQualitativeTemporalConstraintNetwork, mTimepoints);

    mResourceRequirements = Mission::getResourceRequirements(mpMission);
    if(mResourceRequirements.empty())
    {
        throw std::invalid_argument("templ::solvers::csp::TransportNetwork: no resource requirements given");
    }
    breakpointStart()
        << "Requirements:" << std::endl
        << FluentTimeResource::toString(mResourceRequirements, 4)
        << "Timepoints: " << mTimepoints << std::endl
        << mQualitativeTimepoints << std::endl;
    breakpointEnd();

    // update timepoint comparator for intervals
    FluentTimeResource::updateIndices(mResourceRequirements,
            mLocations);

    mModelUsage = Gecode::IntVarArray(*this,
            /*# of models*/ mpMission->getAvailableResources().size()*
            /*# of fluent time services*/mResourceRequirements.size(), 0, mModelPool.getMaxResourceCount());

    mRoleUsage = Gecode::IntVarArray(*this,
            /*width --> col */ mpMission->getRoles().size()* /*height --> row*/ mResourceRequirements.size(),
            0, 1);// Domain 0,1 to represent activation

    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());

    // Limit roles to resource availability
    initializeRoleDistributionConstraints();

    // Mission additional constraints
    applyMissionConstraints();
    applyExtraConstraints();

    // (C) Avoid computation of solutions that are redunant
    // Gecode documentation says however in 8.10.2 that "Symmetry breaking by
    // LDSB is not guaranteed to be complete. That is, a search may still return
    // two distinct solutions that are symmetric."
    //
    Gecode::Symmetries symmetries = identifySymmetries();

    // For each requirement add the min/max and extensional constraints
    // for all overlapping requirements create maximum resource constraints
    Gecode::branch(*this, &TransportNetwork::doPostMinMaxConstraints);

    // Allow only composite agents, i.e. combinations of models, that provide a particular functionality
    Gecode::branch(*this, &TransportNetwork::doPostExtensionalConstraints);

    Gecode::IntAFC modelUsageAfc(*this, mModelUsage, 0.99);
    double modelAfcDecay = mConfiguration.getValueAs<double>("TransportNetwork/search/options/model-usage/afc-decay",0.95);
    modelUsageAfc.decay(*this, modelAfcDecay);
    branch(*this, mModelUsage, Gecode::INT_VAR_AFC_MIN(modelUsageAfc), Gecode::INT_VAL_SPLIT_MIN());
    //Gecode::Gist::stopBranch(*this);

    Gecode::Rnd modelUsageRnd;
    modelUsageRnd.hw();
    branch(*this, mModelUsage, Gecode::INT_VAR_AFC_MIN(modelUsageAfc), Gecode::INT_VAL_RND(modelUsageRnd));

    branch(*this, mModelUsage, Gecode::tiebreak(Gecode::INT_VAR_DEGREE_MAX(),
                                Gecode::INT_VAR_SIZE_MIN()),
                                Gecode::INT_VAL_SPLIT_MIN());
    //Gecode::Gist::stopBranch(*this);

    // Regarding the use of INT_VALUES_MIN() and INT_VALUES_MAX(): "This is
    // typically a poor choice, as none of the alternatives can benefit from
    // propagation that arises when other values of the same variable are tried.
    // These branchings exist for instructional purposes" p.123 Tip 8.2
    // variable which is unassigned and assigned the smallest value
    //branch(*this, mRoleUsage, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_MIN(), symmetries);
    // variable with the smallest domain size first, and assign the smallest
    // value of the selected variable
    //branch(*this, mRoleUsage, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_MIN(), symmetries);
    //branch(*this, mRoleUsage, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_MIN(), symmetries);

    Gecode::IntAFC roleUsageAfc(*this, mRoleUsage, 0.99);
    double roleAfcDecay = mConfiguration.getValueAs<double>("TransportNetwork/search/options/role-usage/afc-decay",0.95);
    roleUsageAfc.decay(*this, roleAfcDecay);
    //branch(*this, mRoleUsage, Gecode::INT_VAR_AFC_MIN(roleUsageAfc), Gecode::INT_VAL_SPLIT_MIN());

    Gecode::Rnd rnd;
    rnd.hw();
    branch(*this, mRoleUsage, Gecode::INT_VAR_AFC_MIN(roleUsageAfc), Gecode::INT_VAL_RND(rnd), symmetries);
    branch(*this, mRoleUsage, Gecode::INT_VAR_RND(rnd), Gecode::INT_VAL_RND(rnd), symmetries);
    branch(*this, mRoleUsage, Gecode::tiebreak(Gecode::INT_VAR_DEGREE_MAX(),
                                Gecode::INT_VAR_SIZE_MIN()),
                                Gecode::INT_VAL_SPLIT_MIN());

    //Gecode::Gist::stopBranch(*this);
    // see 8.14 Executing code between branchers
    Gecode::branch(*this, &TransportNetwork::doPostRoleAssignments);

    //Gecode::Gist::Print<TransportNetwork> p("Print solution");
    //Gecode::Gist::Options options;
    //options.threads = 1;
    //Gecode::Search::Cutoff * c = Gecode::Search::Cutoff::constant(2);
    //options.cutoff = c;
    //options.inspect.click(&p);
    ////Gecode::Gist::bab(this, o);
    //Gecode::Gist::dfs(this, options);


    // General resource constraints
    //  - identify overlapping fts, limit resources for these (TODO: better
    //    identification of overlapping requirements)
    //
    setUpperBoundForConcurrentRequirements();
    // There can be only one assignment per role
    enforceUnaryResourceUsage();
}

void TransportNetwork::doPostMinMaxConstraints(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).initializeMinMaxConstraints();
}

void TransportNetwork::doPostExtensionalConstraints(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).addExtensionalConstraints();
}

void TransportNetwork::doPostRoleAssignments(Gecode::Space& home)
{
    breakpoint("doPostRoleAssignments()");
    static_cast<TransportNetwork&>(home).postRoleAssignments();
}

void TransportNetwork::postRoleAssignments()
{
    (void) status();

    LOG_WARN_S << "Posting Role Assignments: request status" << std::endl
        << modelUsageToString() << std::endl
        << roleUsageToString();

    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    //#############################################
    // construct timelines
    // ############################################
    // 0: l0-t0: {}
    // 1: l1-t0: {2}
    // 2: l0-t1: {4}
    // 3: l1-t1: {}
    // 4: l0-t2: {..}
    // ...
    size_t numberOfFluents = mLocations.size();
    size_t numberOfTimepoints = mTimepoints.size();
    size_t locationTimeSize = numberOfFluents*numberOfTimepoints;

    mActiveRoles = computeActiveRoles();

    LOG_INFO_S << std::endl
        << mTimepoints << std::endl
        << symbols::constants::Location::toString(mLocations);

    assert(!mActiveRoles.empty());
    assert(mTimelines.empty());

    Role::List activeRoles;
    std::vector<uint32_t>::const_iterator rit = mActiveRoles.begin();
    for(; rit != mActiveRoles.end(); ++rit)
    {
        uint32_t roleIndex = *rit;
        const Role& role = mRoles[roleIndex];
        activeRoles.push_back(role);

        // A timeline describes the transitions in space time for a given role
        // A timeline is represented by an adjacency list, pointing from the current node
        // to the next -- given some temporal constraints
        // An empty list assignment means, there is no transition and at
        // maximum there can be one transition

        /// Initialize timelines for all roles, i.e. here the current one
        /// Ajacencylist
        unsigned int minCard = 0;
        unsigned int maxCard = 1;
        Gecode::SetVarArray timeline(*this, locationTimeSize, Gecode::IntSet::empty, Gecode::IntSet(0,locationTimeSize-1),minCard, maxCard);
        mTimelines.push_back(timeline);

        // Setup the basic constraints for the timeline
        // i.e. only edges from one timestep to the next are allowed
        for(size_t t = 0; t < numberOfTimepoints; ++t)
        {
            Gecode::IntVarArray cardinalities(*this, numberOfFluents, 0, 1);
            for(size_t l = 0; l < numberOfFluents; ++l)
            {
                int idx = t*numberOfFluents + l;
                const Gecode::SetVar& edgeActivation = timeline[idx];
                // Use SetView to manipulate the edgeActivation in the
                // timeline
                Gecode::Set::SetView v(edgeActivation);
                // http://www.gecode.org/doc-latest/reference/classGecode_1_1Set_1_1SetView.html
                // set value to 'col' which represents the next target
                // space-time-point
                v.cardMin(*this, 0);
                v.cardMax(*this, 1);
                // exclude space-time-points outside the next step
                v.exclude(*this, 0, (t+1)*numberOfFluents - 1);
                v.exclude(*this, (t+2)*numberOfFluents, numberOfTimepoints*numberOfFluents);

                Gecode::cardinality(*this, edgeActivation, cardinalities[l]);
            }

            // Require exactly one outgoing edge per timestep except for the last
            // cardinality of 1 for sum of cardinalities
            if(t < numberOfTimepoints - 1)
            {
                Gecode::linear(*this, cardinalities, Gecode::IRT_EQ, 1);
            }
        }
        using namespace solvers::temporal;

        // Link the edge activation to the role requirement, i.e. make sure that
        // for each requirement the interval is 'activated'
        for(uint32_t requirementIndex = 0; requirementIndex < mResourceRequirements.size(); ++requirementIndex)
        {
            // Check if the current role (identified by roleIndex) is required to fulfil the
            // requirement
            Gecode::IntVar roleRequirement = roleDistribution(roleIndex, requirementIndex);
            if(!roleRequirement.assigned())
            {
                throw std::runtime_error("TransportNetwork: roleRequirement is not assigned");
            }
            Gecode::IntVarValues var(roleRequirement);
            // Check if role is required, i.e., does it map to the interval
            // then the assigned value is one
            if(var.val() == 1)
            {
                const FluentTimeResource& fts = mResourceRequirements[requirementIndex];
                // index of the location is: fts.fluent
                point_algebra::TimePoint::Ptr from = fts.getInterval().getFrom();
                point_algebra::TimePoint::Ptr to = fts.getInterval().getTo();
                uint32_t fromIndex = getTimepointIndex( from );
                uint32_t toIndex = getTimepointIndex( to );

                // The timeline is now updated for the full interval the
                // requirement is covering
                uint32_t timeIndex = fromIndex;
                for(; timeIndex < toIndex; ++timeIndex)
                {
                    // index of the location is fts.fluent
                    // edge index:
                    // row = timepointIdx*#ofLocations + from-location-offset
                    // col = (timepointIdx + 1) *#ofLocations + to-location-offset
                    //
                    // location (offset) = row % #ofLocations
                    // timepointIndex = (row - location(offset)) / #ofLocations
                    size_t fluentIdx = fts.getFluentIdx();
                    size_t row = FluentTimeIndex::toRowOrColumnIndex(fluentIdx, timeIndex, numberOfFluents, numberOfTimepoints);
                    // Always connect to the next timestep
                    size_t col = FluentTimeIndex::toRowOrColumnIndex(fluentIdx, timeIndex + 1, numberOfFluents, numberOfFluents);


                    LOG_INFO_S << "EdgeActivation for col: " << col << ", row: " << row << " requirement for: " << role.toString() << " roleRequirement: " << roleRequirement;
                    LOG_INFO_S << "Translates to: " << from->toString() << " to " << to->toString();
                    LOG_INFO_S << "Fluent: " << mLocations[fluentIdx]->toString();


                    // constraint between timeline and roleRequirement
                    // if 'roleRequirement' is given, then edgeActivation = col,
                    // else edgeActivation restricted only using standard time
                    LOG_DEBUG_S << "Set SetVar in col: " << col << " and row " << row;
                    Gecode::SetVar& edgeActivation = timeline[row];
                    // Use SetView to manipulate the edgeActivation in the
                    // timeline
                    Gecode::Set::SetView v(edgeActivation);
                    // http://www.gecode.org/doc-latest/reference/classGecode_1_1Set_1_1SetView.html
                    // set value to 'col'
                    // workaround to set {col} as target for this edge
                    v.intersect(*this, col,col);
                    v.cardMin(*this, 1);
                    v.cardMax(*this, 1);

                    // now limit parallel values of the edge target
                    // since (due to the existing path constraint) the next
                    // edge has to have this target as source
                    //
                    // do this only if we have not reached end of time horizon
                    if(timeIndex < numberOfTimepoints - 1)
                    {
                        size_t offset = (timeIndex+1)*numberOfFluents;
                        for(size_t f = 0; f < numberOfFluents; ++f)
                        {
                            size_t idx = offset + f;
                            if(idx != col)
                            {
                                Gecode::SetVar& excludeVar = timeline[idx];
                                Gecode::Set::SetView excludeView(excludeVar);

                                excludeView.cardMax(*this, 0);
                            }
                        }
                    } // end of time horizon
                }
            } // end if(v.val() == 1)
        } // for loop requirements
    } // for loop active roles

    mActiveRoleList = activeRoles;
    assert(!mActiveRoleList.empty());
    // Construct the basic timeline
    //
    // Map role requirements back to activation in general network
    // requirement = location t0--tN, role-0, role-1
    //
    // foreach involved role
    //     foreach requirement
    //          from lX,t0 --> tN
    //              request edge activation (referring to the role is active during that interval)
    //              by >= value of the requirement( which is typically 0 or 1),
    //              whereas activation can be 0 or 1 as well
    //
    // Compute a network with proper activation
    //branch(*this, &TransportNetwork::postRoleTimelines);
    std::vector<int32_t> supplyDemand;
    if( mConfiguration.getValueAs<bool>("TransportNetwork/search/options/timeline-brancher/supply-demand",false) )
    {
        for(uint32_t roleIdx = 0; roleIdx < mActiveRoles.size(); ++roleIdx)
        {
            const Role& role = mRoles[ mActiveRoles[roleIdx] ];
            using namespace moreorg::facades;
            Robot robot = Robot::getInstance(role.getModel(), mAsk);
            if(robot.isMobile())
            {
                uint32_t transportCapacity = robot.getTransportCapacity();
                supplyDemand.push_back(transportCapacity);
            } else {
                uint32_t transportDemand = robot.getTransportDemand();
                if(transportDemand == 0)
                {
                    throw std::invalid_argument("templ::solvers::csp::TransportNetwork: " +  role.getModel().toString() + " has"
                            " a transportSupplyDemand of 0 -- must be either positive of negative integer");
                }
                supplyDemand.push_back(-transportDemand);
            }
        }
        mSupplyDemand = supplyDemand;
        assert(!supplyDemand.empty());
    }

    Gecode::Rnd rnd;
    rnd.hw();
    double timelineAfcDecay = mConfiguration.getValueAs<double>("TransportNetwork/search/options/timeline-brancher/afc-decay");
    size_t numberOfLocations = mLocations.size();
    for(size_t i = 0; i < mActiveRoles.size(); ++i)
    {
        const Role& role = mActiveRoleList[i];

        propagators::isPath(*this, mTimelines[i], role.toString(),
                numberOfTimepoints, numberOfLocations);

        // Only branch on the mobile systems
        using namespace moreorg::facades;
        Robot robot = Robot::getInstance(role.getModel(), mAsk);
        if(robot.isMobile())
        {
            Gecode::SetAFC timelineUsageAfc(*this, mTimelines[i], timelineAfcDecay);
            branch(*this, mTimelines[i],Gecode::SET_VAR_AFC_MIN(timelineAfcDecay), Gecode::SET_VAL_RND_EXC(rnd));
            branch(*this, mTimelines[i],Gecode::SET_VAR_RND(rnd),Gecode::SET_VAL_RND_EXC(rnd));
            branch(*this, mTimelines[i], Gecode::tiebreak(
                        Gecode::SET_VAR_DEGREE_MAX(),
                        Gecode::SET_VAR_SIZE_MIN()),
                    Gecode::SET_VAL_RND_EXC(rnd));
        }
    }

    // Record the minimal required timeline for this role, before branching
    // expansion of the timeline takes place
    mMinRequiredTimelines = getTimelines();

    // BEGIN LOCATION ACCESS
    applyAccessConstraints(mTimelines,
            numberOfTimepoints,
            numberOfLocations,
            mActiveRoleList);
    // END LOCATION ACCESS
    // Only the check whether a feasible approach is to use a heuristic
    // to draw system by supply demand
    //branchTimelines(*this, mTimelines, mSupplyDemand);
    Gecode::branch(*this,&TransportNetwork::doPostMinCostFlow);
    Gecode::Gist::stopBranch(*this);
}


void TransportNetwork::doPostMinCostFlow(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).postMinCostFlow();
}

void TransportNetwork::postMinCostFlow()
{
    save();

    try {
        breakpointStart()
            << "Remaining flaws computation: " << mMinCostFlowFlaws.size() << std::endl
            << "     cost: " << mCost << std::endl
            << "     flaws: " << mNumberOfFlaws << std::endl;
        breakpointEnd();

        std::string solver =
            mConfiguration.getValueAs<std::string>("TransportNetwork/search/options/lp/solver","CBC_SOLVER");

        namespace ga = graph_analysis::algorithms;
        ga::LPSolver::Type solverType = ga::LPSolver::UNKNOWN_LP_SOLVER;
        for(const std::pair<ga::LPSolver::Type, std::string>& p :
                graph_analysis::algorithms::LPSolver::TypeTxt)
        {
            if(p.second == solver)
            {
                solverType = p.first;
                break;
            }
        }

        double feasibilityTimeoutInMs = 1000*mConfiguration.getValueAs<double>("TransportNetwork/search/options/coalition-feasibility/timeout_in_s",1);

        std::map<Role, RoleTimeline> expandedTimelines = getTimelines();


        LOG_INFO_S << "Min required: " <<
            RoleTimeline::toString(mMinRequiredTimelines,4,false);
        LOG_INFO_S << "Expanded: " <<
            RoleTimeline::toString(expandedTimelines,4,false);

        std::pair< std::map<Role, csp::RoleTimeline>, std::map<Role, csp::RoleTimeline> >
            key(expandedTimelines, mMinRequiredTimelines);

        transshipment::MinCostFlow minCostFlow(expandedTimelines,
                mMinRequiredTimelines,
                mLocations,
                mTimepoints,
                mAsk,
                mpMission->getLogger(),
                solverType,
                feasibilityTimeoutInMs);

        breakpointStart()
            << "Min cost flow to start" << std::endl;
        breakpointEnd();

        FlowSolutions::iterator it = msMinCostFlowSolutions.find(key);
        if(it == msMinCostFlowSolutions.end())
        {
            std::vector<transshipment::Flaw> flaws = minCostFlow.run();

            breakpointStart()
                << "Min cost flow to start" << std::endl;
            breakpointEnd();

            transshipment::FlowNetwork flowNetwork = minCostFlow.getFlowNetwork();
            mMinCostFlowSolution = flowNetwork.getSpaceTimeNetwork();

            if(flaws.empty())
            {
                if(propagateImmobileAgentConstraints(mMinCostFlowSolution) ==
                        Gecode::ES_FAILED)
                {
                    LOG_WARN_S << "Immobile agent constraints not maintained by"
                        << " local search";
                    return;
                }
            }

            flowNetwork.save();

            // store all flaws
            mMinCostFlowFlaws = flaws;

            if(mConfiguration.getValueAs<bool>("TransportNetwork/search/options/lp/cache-solution",
                        false))
            {
                msMinCostFlowSolutions[key] = FlowSolutionValue(flaws, mMinCostFlowSolution);
            }
        } else {
            breakpointStart()
                << "Found existing solution .. (skipping recomputation and taking from cache)" << std::endl;
            breakpointEnd();

            mMinCostFlowFlaws = it->second.first;
            mMinCostFlowSolution = it->second.second;
        }
        // compute all feasible resolution that might allow
        // to improve the solution
        mFlawResolution.prepare(mMinCostFlowFlaws);

        std::cout << "Session " << mpMission->getLogger()->getSessionId() << ": remaining flaws: " << mMinCostFlowFlaws.size() << std::endl;
        breakpointStart()
            << "Remaining flaws: " << mMinCostFlowFlaws.size() << std::endl;
        breakpointEnd();

        // Set flaws as current cost of this solution
        rel(*this, mCost, Gecode::IRT_EQ, mMinCostFlowFlaws.size());
        rel(*this, mNumberOfFlaws, Gecode::IRT_EQ, mMinCostFlowFlaws.size());

        mSolutionAnalysis = solvers::SolutionAnalysis(mpMission, mMinCostFlowSolution, mConfiguration);
        mSolutionAnalysis.analyse();

        // Set flaws as well
        bool allowFlaws = mConfiguration.getValueAs<bool>("TransportNetwork/search/options/allow-flaws", true);
        if(!mMinCostFlowFlaws.empty() && !allowFlaws)
        {
            this->fail();
            return;
        }

    } catch(const std::runtime_error& e)
    {
        // Min cost flow optimization or negative cycle checking
        LOG_WARN_S << "templ::solvers::csp::TransportNetwork: could not find"
            << " solution: " << e.what();
        this->fail();
        return;
    }
}

void TransportNetwork::doPostTimelines(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).postTimelines();
}

void TransportNetwork::postTimelines()
{
    breakpointStart()
        << "Post timelines" << std::endl;
    breakpointEnd();

    branchTimelines(*this, mTimelines, mSupplyDemand);
}

std::string TransportNetwork::toString() const
{
    std::stringstream ss;
    ss << "TransportNetwork: #" << std::endl;
    ss << "    Timepoints: " << mQualitativeTimepoints << std::endl;
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, mModelPool.size(), mResourceRequirements.size());
    for(size_t m = 0; m < mModelPool.size(); ++m)
    {
        const owlapi::model::IRI& model = getResourceModelFromIndex(m);
        ss << std::setw(30) << std::left << model.getFragment() << ": ";
        for(size_t i = 0; i < mResourceRequirements.size(); ++i)
        {
            ss << std::setw(10) << std::left << resourceDistribution(m,i);
        }
        ss << std::endl;
    }

    Gecode::Matrix<Gecode::IntVarArray> rolesDistribution(mRoleUsage, mRoles.size(), mResourceRequirements.size());
    size_t width = 30;
    for(size_t m = 0; m < mRoles.size(); ++m)
    {
        width = std::min(mRoles[m].toString().size() + 5, width);
    }

    for(size_t m = 0; m < mRoles.size(); ++m)
    {
        ss << std::setw(width) << mRoles[m].toString() << ": ";
        for(size_t i = 0; i < mResourceRequirements.size(); ++i)
        {
            ss << std::setw(10) << std::left << rolesDistribution(m,i);
        }
        ss << std::endl;
    }
    try {
        for(size_t i = 0; i < mTimelines.size(); ++i)
        {
            ss << mActiveRoleList[i].toString() << std::endl;
            ss << Formatter::toString(mTimelines[i], mLocations, mTimepoints) << std::endl;
        }

    } catch(const std::exception& e)
    {
        ss << "Number of timelines: n/a -- " << e.what() << std::endl;
        ss << "Current timelines: n/a " << std::endl;
    }

    //ss << "Capacities: " << std::endl << Formatter::toString(mCapacities,
    //        toPtrList<Symbol,symbols::constants::Location>(mLocations),
    //        toPtrList<Variable, temporal::point_algebra::TimePoint>(mTimepoints)
    //        ) << std::endl;

    return ss.str();
}

std::string TransportNetwork::modelUsageToString() const
{
    size_t firstcolumnwidth = 30;
    size_t columnwidth = 20;
    std::stringstream ss;
    ss << "Model usage:" << std::endl;
    ss << std::setw(firstcolumnwidth) << std::right << "    FluentTimeResource: ";
    for(size_t r = 0; r < mResourceRequirements.size(); ++r)
    {
        const FluentTimeResource& fts = mResourceRequirements[r];
        /// construct string for proper alignment
        std::string s = fts.getFluent()->getInstanceName();
        s += "@[" + fts.getInterval().toString(0,true) + "]";

        ss << std::setw(columnwidth) << std::left << s;
    }
    ss << std::endl;

    int modelIndex = 0;
    moreorg::ModelPool::const_iterator cit = mModelPool.begin();
    for(; cit != mModelPool.end(); ++cit, ++modelIndex)
    {
        const owlapi::model::IRI& model = cit->first;
        ss << std::setw(firstcolumnwidth) << std::left << model.getFragment() << ": ";
        for(size_t r = 0; r < mResourceRequirements.size(); ++r)
        {
            ss << std::setw(columnwidth) << mModelUsage[r*mModelPool.size() + modelIndex] << " ";
        }
        ss << std::endl;
    }
    return ss.str();
}

std::string TransportNetwork::roleUsageToString() const
{
    return Formatter::toString(mRoleUsage, mRoles, mResourceRequirements);
}

std::string TransportNetwork::toString(const std::vector<Gecode::IntVarArray>& timelines) const
{
    std::vector<uint32_t> activeRoles = getActiveRoles();
    std::vector<std::string> labels;
    for(size_t i = 0; i < timelines.size(); ++i)
    {
        labels.push_back( mRoles[ activeRoles[i] ] .toString());
    }
    return Formatter::toString(timelines,
            toPtrList<Symbol,symbols::constants::Location>(mLocations),
            toPtrList<Variable, temporal::point_algebra::TimePoint>(mTimepoints),
            labels);
}

uint32_t TransportNetwork::getTimepointIndex(const temporal::point_algebra::TimePoint::Ptr& timePoint) const
{
    using namespace templ::solvers::temporal;

    std::vector<point_algebra::TimePoint::Ptr>::const_iterator timepointIt = std::find(mTimepoints.begin(), mTimepoints.end(), timePoint);
    if(timepointIt != mTimepoints.end())
    {
        return timepointIt - mTimepoints.begin();
    }
    throw std::invalid_argument("templ::solvers::csp::TransportNetwork::getTimepointIdx: unknown timepoint '" + timePoint->toString() + "' given");
}

std::ostream& operator<<(std::ostream& os, const TransportNetwork::Solution& solution)
{
    os << solution.toString();
    return os;
}

std::ostream& operator<<(std::ostream& os, const TransportNetwork::SolutionList& solutions)
{
    TransportNetwork::SolutionList::const_iterator cit = solutions.begin();
    os << std::endl << "BEGIN SolutionList (#" << solutions.size() << " solutions)" << std::endl;
    size_t count = 0;
    for(; cit != solutions.end(); ++cit)
    {
        os << "#" << count++ << " ";
        os << *cit;
    }
    os << "END SolutionList" << std::endl;
    return os;
}

void TransportNetwork::save(const std::string& _filename) const
{
    std::string filename = _filename;
    if(filename.empty())
    {
        base::Time timestamp = base::Time::now();
        filename = mpMission->getLogger()->filename(timestamp.toString(base::Time::Seconds) + "-transport-network.status");
    }
    std::ofstream file;
    file.open(filename );
    file << toString();
    file.close();
}

Gecode::ExecStatus TransportNetwork::propagateImmobileAgentConstraints(const SpaceTime::Network& network)
{
    std::map<symbols::constants::Location::Ptr, size_t> mLocationIdxMap;
    size_t numberOfLocations = mLocations.size();
    for(size_t idx = 0; idx < numberOfLocations; ++idx)
    {
        mLocationIdxMap[ mLocations[idx] ] = idx;
    }
    std::map<temporal::point_algebra::TimePoint::Ptr, size_t> mTimepointIdxMap;
    size_t numberOfTimepoints = mTimepoints.size();
    for(size_t idx = 0; idx < numberOfTimepoints; ++idx)
    {
        mTimepointIdxMap[ mTimepoints[idx] ] = idx;
    }

    Role::List activeImmobileRoles;
    std::vector<size_t> activeImmobileRolesIdx;
    for(size_t idx = 0; idx < mActiveRoleList.size(); ++idx)
    {
        const Role& role = mActiveRoleList[idx];
        using namespace moreorg::facades;
        Robot robot = Robot::getInstance(role.getModel(), mAsk);
        if(!robot.isMobile())
        {
            activeImmobileRoles.push_back(role);
            activeImmobileRolesIdx.push_back(idx);
        }
    }

    using namespace graph_analysis;
    BaseGraph::Ptr graph = network.getGraph();
    VertexIterator::Ptr vertexIt = graph->getVertexIterator();
    while(vertexIt->next())
    {
        SpaceTime::Network::tuple_t::Ptr tuple =
            dynamic_pointer_cast<SpaceTime::Network::tuple_t>(vertexIt->current());

        Role::Set roles = tuple->getRoles(RoleInfo::ASSIGNED);
        for(const Role& role : roles)
        {
            for(size_t idx : activeImmobileRolesIdx)
            {
                if(role == activeImmobileRoles[idx])
                {
                    if(updateTimeline(idx,
                                mLocationIdxMap[tuple->first()],
                                mTimepointIdxMap[tuple->second()],
                                numberOfLocations,
                                numberOfTimepoints) == Gecode::ES_FAILED)
                    {
                        LOG_WARN_S << "Constraint could not be maintained for"
                            << " role '" << role.toString() << "' at "
                            << " location " << tuple->first()->getInstanceName()
                            << " timepoint " << tuple->second()->getLabel();
                        return Gecode::ES_FAILED;
                    }
                }
            }
        }
    }
    return Gecode::ES_OK;
}

Gecode::ExecStatus TransportNetwork::updateTimeline(size_t timelineIdx,
        size_t locationIdx,
        size_t timepointIdx,
        size_t numberOfLocations,
        size_t numberOfTimepoints)
{
    if(timepointIdx == 0 || timelineIdx >= numberOfTimepoints)
    {
        return Gecode::ES_OK;
    }

    for(size_t i = 0; i < numberOfLocations; ++i)
    {
        if(i != locationIdx)
        {
            Gecode::Set::SetView view(mTimelines[timelineIdx][timepointIdx*numberOfLocations + i]);
            // only the given locations should have an outgoing edge,
            // thus all other locations with empty sets
            view.cardMax(*this, 0);
        }

        Gecode::SetVar var = mTimelines[timelineIdx][(timepointIdx-1)*numberOfLocations + i];

        // Already assigned value have been propagated before local search,
        // so no need to revalidate
        if(var.assigned())
        {
            continue;
        }

        Gecode::Set::SetView view(var);
        if(locationIdx > 0)
        {
            if(view.cardMax() != 0)
            {
                size_t lower = timepointIdx*numberOfLocations;
                size_t upper = timepointIdx*numberOfLocations + locationIdx -1;
                for(size_t l = lower; l <= upper; ++l)
                {
                    GECODE_ME_CHECK(view.exclude(*this, l));
                }
            }
        }
        if(locationIdx < numberOfLocations - 1)
        {
            if(view.cardMax() != 0)
            {
                size_t lower = timepointIdx*numberOfLocations + locationIdx + 1;
                size_t upper = timepointIdx*numberOfLocations + numberOfLocations + 1;
                for(size_t l = lower; l <= upper; ++l)
                {
                    GECODE_ME_CHECK(view.exclude(*this, l));
                }
            }
        }
    }
    return Gecode::ES_OK;
}

void TransportNetwork::breakpoint(const std::string& msg)
{
    if(msInteractive)
    {
        std::cout << msg << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }
}

std::ostream& TransportNetwork::breakpointStart()
{
    mInteractiveMessageStream.clear();
    return mInteractiveMessageStream;
}

void TransportNetwork::breakpointEnd()
{
    breakpoint(mInteractiveMessageStream.str());
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
