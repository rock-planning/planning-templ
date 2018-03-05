#include "TransportNetwork.hpp"
#include <base-logging/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include <gecode/minimodel.hh>
#include <gecode/set.hh>
#include <gecode/gist.hh>
#include <gecode/search.hh>
#include <gecode/search/meta/rbs.hh>

#include <iomanip>
#include <fstream>
#include <Eigen/Dense>

#include <organization_model/Algebra.hpp>
#include <organization_model/vocabularies/OM.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/symbols/object_variables/LocationCardinality.hpp>
#include <templ/SpaceTime.hpp>
#include <organization_model/facades/Robot.hpp>

#include "ConstraintMatrix.hpp"
#include "branchers/TimelineBrancher.hpp"
#include "propagators/IsPath.hpp"
#include "propagators/IsValidTransportEdge.hpp"
#include "propagators/MultiCommodityFlow.hpp"
#include "utils/Formatter.hpp"
#include "utils/Converter.hpp"
#include "../../utils/CSVLogger.hpp"
#include <graph_analysis/GraphIO.hpp>
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

            const organization_model::ModelPool& modelPool = cit->second;
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
        ss << hspace  << SpaceTime::toString(mTimelines);
    }
    return ss.str();
}

SpaceTime::Network TransportNetwork::Solution::toNetwork() const
{
    return SpaceTime::toNetwork(mLocations, mTimepoints, mTimelines);
}

TransportNetwork::SearchState::SearchState(const Mission::Ptr& mission)
    : mpMission(mission)
    , mpInitialState(NULL)
    , mpSearchEngine(NULL)
    , mType(OPEN)
{
    assert(mpMission);
    mpInitialState = TransportNetwork::Ptr(new TransportNetwork(mpMission));
    mpSearchEngine = TransportNetwork::BABSearchEnginePtr(new Gecode::BAB<TransportNetwork>(mpInitialState.get()));
}

TransportNetwork::SearchState::SearchState(const TransportNetwork::Ptr& transportNetwork,
        const TransportNetwork::BABSearchEnginePtr& searchEngine)
    : mpMission(transportNetwork->mpMission)
    , mpInitialState(transportNetwork)
    , mpSearchEngine(searchEngine)
    , mType(OPEN)
{
    assert(mpMission);
    assert(mpInitialState);
    if(!mpSearchEngine)
    {
        mpSearchEngine = TransportNetwork::BABSearchEnginePtr(new Gecode::BAB<TransportNetwork>(mpInitialState.get()));
    }
}

TransportNetwork::SearchState TransportNetwork::SearchState::next() const
{
    if(!getInitialState())
    {
        throw std::runtime_error("templ::solvers::csp::TransportNetwork::SearchState::next: "
                " next() called on an unitialized search state");
    }

    SearchState searchState(mpInitialState, mpSearchEngine);
    TransportNetwork* solvedDistribution = mpSearchEngine->next();
    if(solvedDistribution)
    {
        searchState.mSolution = solvedDistribution->getSolution();
        searchState.mType = SUCCESS;
        delete solvedDistribution;
    } else {
        searchState.mType = FAILED;
    }
    return searchState;
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
    if(msInteractive)
    {
        std::cout << "BEGIN next()" << std::endl;
        std::cout << "Press ENTER to continue..." << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }

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

    if(msInteractive)
    {
        std::cout << "next():" << std::endl;
        std::cout << "    # flaws: " << mMinCostFlowFlaws.size() << std::endl;
        std::cout << "    # resolution options: " << mFlawResolution.remainingDraws().size() << std::endl;
        std::cout << "    # required resolution options: " << mRequiredResolutionOptions.size() << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }

    namespace ga = graph_analysis::algorithms;

    FlawResolution::EvaluationList evalList = FlawResolution::selectBestResolution(*this, lastSpace, lastSpace.cost().val(), mFlawResolution.getResolutionOptions());
    if(evalList.empty())
    {
        std::cout << "    # no applicable resolvers -- failing search" << std::endl;
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

    for(const FlawResolution::Evaluation& e : evalList)
    {
        std::cout << "Requiring flaw which improves with delta: " << e.second;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        mRequiredResolutionOptions.push_back( e.first );
    //mRequiredResolutionOptions.insert(mRequiredResolutionOptions.begin(), resolutionOptions.begin(), resolutionOptions.end());
    }

    for(const FlawResolution::ResolutionOption& resolutionOption : mRequiredResolutionOptions)
    {
        FlawResolution::applyResolutionOption(*this, lastSpace, resolutionOption);

        // Recompute unary resource usage since constraints have changed
        enforceUnaryResourceUsage();

        if(msInteractive)
        {
            std::cout << "END next():" << std::endl;
            std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }


    }

    //if(msInteractive)
    //{
    //    std::cout << "Relax model and role usage" << std::endl;
    //    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    //}
    //Gecode::Rnd r(1U);
    //Gecode::relax(*this, mModelUsage, lastSpace.mModelUsage, r, 0.7);
    //Gecode::relax(*this, mRoleUsage, lastSpace.mRoleUsage, r, -1.7);

    //std::cout << "Before relax status:" << roleUsageToString() << std::endl;
    //std::cout << "After status:" << modelUsageToString() << std::endl;
    //std::cout << "After status:" << roleUsageToString() << std::endl;
    //std::cout << "Model Usage: " << mModelUsage;
    //std::cout << "Role Usage: " << mRoleUsage;
}

// Constrain function is called with the last solution found as argument (if a
// solution has already been found
void TransportNetwork::constrain(const Gecode::Space& lastSpace)
{
    const TransportNetwork& lastTransportNetwork = static_cast<const TransportNetwork&>(lastSpace);

    if(msInteractive)
    {
        std::cout << "constrain()" << std::endl;
        std::cout << "Last state: " << std::endl;
        std::cout << "    # cost: "<< lastTransportNetwork.mCost.val() << std::endl;
        std::cout << "    # flaws: "<< lastTransportNetwork.mMinCostFlowFlaws.size() << std::endl;
        std::cout << "    # resolution options: " << lastTransportNetwork.mFlawResolution.remainingDraws().size() << std::endl;

        std::cout << "Press ENTER to continue..." << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }


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

    if(msInteractive)
    {
        std::cout << "constrainSlave()" << std::endl;
        std::cout << "Last state: " << std::endl;
        std::cout << "    # flaws: "<< lastTransportNetwork.mMinCostFlowFlaws.size() << std::endl;
        std::cout << "    # resolution options: " << lastTransportNetwork.mFlawResolution.remainingDraws().size() << std::endl;

        std::cout << "Press ENTER to continue..." << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }

    rel(*this, cost(), Gecode::IRT_LE, lastTransportNetwork.cost().val());

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

    std::cout << "slave() with restarts: " << mi.restart() << std::endl;
    if(msInteractive)
    {
        std::cout << "Press ENTER to continue..." << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }

    if(mi.type() == Gecode::MetaInfo::RESTART)
    {
        if(!mi.last())
        {
            // the previous call with initialize the selected draw
            if(msInteractive)
            {
                std::cout << "No last space available, thus slave search is complete" << std::endl;
                std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
            }
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
            if(msInteractive)
            {
                std::cout << "Flaw resolution: options are exhausted slave search is complete" << std::endl;
                std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
            }
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

    std::cout << "Solution analysis" << std::endl;
    solvers::SolutionAnalysis sa(mission, solution.getMinCostFlowSolution());
    sa.analyse();
    try {
        sa.save();
    } catch(const std::exception& e)
    {
        LOG_WARN_S << "Saving solution analysis failed: -- " << e.what();
    }

    std::cout << "    Required roles: " << Role::toString(sa.getRequiredRoles()) << std::endl;

    graph_analysis::BaseGraph::Ptr hyperGraph = sa.toHyperGraph();
    try {
        std::stringstream ss;
        ss << "transport-network-solution-hypergraph" << i << ".dot";
        filename = mission->getLogger()->filename(ss.str());
        graph_analysis::io::GraphIO::write(filename, hyperGraph);
    } catch(const std::exception& e)
    {
        LOG_WARN_S << "Saving file " << filename << " failed: -- " << e.what();
    }

}

TransportNetwork::ModelDistribution TransportNetwork::getModelDistribution() const
{
    ModelDistribution solution;
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, mModelPool.size(), mResourceRequirements.size());

    // Check if resource requirements holds
    for(size_t i = 0; i < mResourceRequirements.size(); ++i)
    {
        organization_model::ModelPool modelPool;
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

SpaceTime::Timelines TransportNetwork::getTimelines() const
{
    for(size_t i = 0; i < mActiveRoleList.size(); ++i)
    {
        LOG_WARN_S << "Active role: " << i << " of " << mActiveRoleList.size() << " " << mActiveRoleList[i].toString() << std::endl
            << Formatter::toString(mTimelines[i], mLocations.size());
    }

    bool doThrow = false;
    return TypeConversion::toTimelines(mActiveRoleList, mTimelines, mLocations, mTimepoints, doThrow);
}

TransportNetwork::TransportNetwork()
    : Gecode::Space()
    , Solver(Solver::CSP_TRANSPORT_NETWORK)
    , mAsk()
{}

TransportNetwork::TransportNetwork(const templ::Mission::Ptr& mission, const Configuration& configuration)
    : Gecode::Space()
    , Solver(Solver::CSP_TRANSPORT_NETWORK)
    , mpMission(mission)
    , mModelPool(mpMission->getAvailableResources())
    , mAsk(mpMission->getOrganizationModel(), mpMission->getAvailableResources(), true)
    , mResources(mpMission->getRequestedResources())
    , mIntervals(mpMission->getTimeIntervals())
    , mTimepoints(mpMission->getUnorderedTimepoints())
    , mLocations(mpMission->getLocations())
    , mResourceRequirements(Mission::getResourceRequirements(mpMission))
    , mQualitativeTimepoints(*this, mpMission->getQualitativeTemporalConstraintNetwork()->getTimepoints().size(), 0, mpMission->getQualitativeTemporalConstraintNetwork()->getTimepoints().size()-1)
    , mModelUsage(*this, /*# of models*/ mpMission->getAvailableResources().size()*
            /*# of fluent time services*/mResourceRequirements.size(), 0, mModelPool.getMaxResourceCount()) // maximum number of model at that point
    , mAvailableModels(mpMission->getModels())
    , mRoleUsage(*this, /*width --> col */ mission->getRoles().size()* /*height --> row*/ mResourceRequirements.size(), 0, 1) // Domain 0,1 to represent activation
    , mRoles(mission->getRoles())
    , mCost(*this,0, Gecode::Int::Limits::max)
    , mNumberOfFlaws(*this,0, Gecode::Int::Limits::max)
    , mConfiguration(configuration)
    , mUseMasterSlave(false)
    , mpCurrentMaster(NULL)
   // , mCapacities(*this, (mLocations.size()+1)*(mLocations.size()+1)*mTimepoints.size()*mTimepoints.size(), 0, Gecode::Int::Limits::max)
{
    assert( mpMission->getOrganizationModel() );
    assert(!mIntervals.empty());

    if(mResourceRequirements.empty())
    {
        throw std::invalid_argument("templ::solvers::csp::TransportNetwork: no resource requirements given");
    }

    LOG_INFO_S << "TransportNetwork CSP Problem Construction" << std::endl
    << "    requested resources: " << mResources << std::endl
    << "    intervals: " << mIntervals.size() << std::endl
    << "    # requirements: " << mResourceRequirements.size() << std::endl;

    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());

    // Limit roles to resource availability
    initializeRoleDistributionConstraints();
    // Mission constraints
    applyMissionConstraints();

    // (C) Avoid computation of solutions that are redunant
    // Gecode documentation says however in 8.10.2 that "Symmetry breaking by
    // LDSB is not guaranteed to be complete. That is, a search may still return
    // two distinct solutions that are symmetric."
    //
    Gecode::Symmetries symmetries = identifySymmetries();

    // Allow branching of temporal constraint network
    // Initialize constraint network after mQualitativeTimepoints has been
    // properly constructed -- otherwise we will trigger segfaults
    mTemporalConstraintNetwork = TemporalConstraintNetworkBase(*mpMission->getQualitativeTemporalConstraintNetwork(),*this, mQualitativeTimepoints);
    // makeing sure we get a fully assigned temporal constraint network, i.e.
    // one without gaps before we proceed
    Gecode::Rnd temporalNetworkRnd;
    temporalNetworkRnd.hw();
    Gecode::assign(*this, mQualitativeTimepoints, Gecode::INT_ASSIGN_RND(temporalNetworkRnd));
    Gecode::branch(*this, &TransportNetwork::doPostTemporalConstraints);

    // For each requirement add the min/max and extensional constraints
    // for all overlapping requirements create maximum resource constraints
    Gecode::branch(*this, &TransportNetwork::doPostMinMaxConstraints);

    // Allow only composite agents, i.e. combinations of models, that provide a particular functionality
    Gecode::branch(*this, &TransportNetwork::doPostExtensionalContraints);

    Gecode::IntAFC modelUsageAfc(*this, mModelUsage, 0.99);
    double modelAfcDecay = mConfiguration.getValueAs<double>("TransportNetwork/search/options/model-usage/afc-decay",0.95);
    modelUsageAfc.decay(*this, modelAfcDecay);
    branch(*this, mModelUsage, Gecode::INT_VAR_AFC_MIN(modelUsageAfc), Gecode::INT_VAL_SPLIT_MIN());
    //Gecode::Gist::stopBranch(*this);

    Gecode::Rnd modelUsageRnd;
    modelUsageRnd.hw();
    branch(*this, mModelUsage, Gecode::INT_VAR_AFC_MIN(modelUsageAfc), Gecode::INT_VAL_RND(modelUsageRnd));
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

}


void TransportNetwork::initializeMinMaxConstraints()
{
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());

    // For debugging purposes
    ConstraintMatrix constraintMatrix(mAvailableModels);

    // Part (A)
    {
        using namespace solvers::temporal;
        LOG_INFO_S << mAsk.toString();
        LOG_DEBUG_S << "Involved services: " << owlapi::model::IRI::toString(mServices, true);
        {
            std::vector<FluentTimeResource>::const_iterator fit = mResourceRequirements.begin();
            for(; fit != mResourceRequirements.end(); ++fit)
            {
                const FluentTimeResource& fts = *fit;

                LOG_DEBUG_S << "(A) Define requirement: " << fts.toString()
                            << "        available models: " << mAvailableModels << std::endl;

                // row: index of requirement
                // col: index of model type
                size_t requirementIndex = fit - mResourceRequirements.begin();
                for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
                {
                    Gecode::IntVar v = resourceDistribution(mi, requirementIndex);

                    {
                        // default min requirement is 0 for a model
                        uint32_t minCardinality = 0;
                        /// Consider resource cardinality constraint
                        /// Check what is set for the given model
                        LOG_DEBUG_S << "Check min cardinality for " << mAvailableModels[mi];
                        organization_model::ModelPool::const_iterator cardinalityIt = fts.getMinCardinalities().find( mAvailableModels[mi] );
                        if(cardinalityIt != fts.getMinCardinalities().end())
                        {
                            minCardinality = cardinalityIt->second;
                            LOG_DEBUG_S << "Found resource cardinality constraint: " << std::endl
                                << "    " << mAvailableModels[mi] << ": minCardinality " << minCardinality;
                        }
                        constraintMatrix.setMin(requirementIndex, mi, minCardinality);
                        rel(*this, v, Gecode::IRT_GQ, minCardinality);
                    }

                    uint32_t maxCardinality = mModelPool[ mAvailableModels[mi] ];
                    // setting the upper bound for this model and this service
                    // based on what the model pool can provide
                    LOG_DEBUG_S << "requirement: " << requirementIndex
                        << ", model: " << mi
                        << " IRT_GQ 0 IRT_LQ: " << maxCardinality;
                    constraintMatrix.setMax(requirementIndex, mi, maxCardinality);
                    rel(*this, v, Gecode::IRT_LQ, maxCardinality);
                }

                // there can be no empty assignment for resource requirement
                rel(*this, sum( resourceDistribution.row(requirementIndex) ) > 0);

                // This can be equivalently modelled using a linear constraint
                // Gecode::IntArgs c(mAvailableModels.size());
                // for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
                //    c[mi] = 1;
                // linear(*this, c, resourceDistribution.row(requirementIndex), Gecode::IRT_GR, 0);
            }
        }

        LOG_INFO_S << constraintMatrix.toString();
        std::cout << "InitializeMinMax: final constraint matrix: " << constraintMatrix.toString() << std::endl;
    }
}

void TransportNetwork::addExtensionalConstraints()
{
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());
        std::map<uint32_t, Gecode::TupleSet> extensionalConstraints;

   std::vector<FluentTimeResource>::const_iterator fit = mResourceRequirements.begin();
   for(; fit != mResourceRequirements.end(); ++fit)
   {
        const FluentTimeResource& ftr = *fit;
        LOG_DEBUG_S << "(A) Define requirement: " << ftr.toString()
                    << "        available models: " << mAvailableModels << std::endl;

        // row: index of requirement
        // col: index of model type
        size_t requirementIndex = fit - mResourceRequirements.begin();

        // Prepare the extensional constraints, i.e. specifying the allowed
        // combinations for each requirement
        organization_model::ModelPool::Set allowedCombinations = ftr.getDomain();
        if(allowedCombinations.empty())
        {
            LOG_WARN_S << "No allowed combinations available with the given constraints: failing this space";
            this->failed();
            return;
        }
        LOG_DEBUG_S << "ExtensionalConstraints: add ftr: " << ftr.toString();
        appendToTupleSet(extensionalConstraints[requirementIndex], allowedCombinations);
   }

   // use the prepared list of extensional constraints to activate the
   // constraints
   for(std::pair<uint32_t, Gecode::TupleSet> pair : extensionalConstraints)
   {
       uint32_t requirementIndex = pair.first;
       Gecode::TupleSet& tupleSet = pair.second;
       // one a tuple set has been fully defined it has to be finalized
       tupleSet.finalize();
       extensional(*this, resourceDistribution.row(requirementIndex), tupleSet);
   }
}


void TransportNetwork::setUpperBoundForConcurrentRequirements()
{
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());

    // Part (B) General resource constraints
    // - identify overlapping fts, limit resources for these
    {
        // Set of available models: mModelPool
        // Make sure the assignments are within resource bounds for concurrent requirements
        std::vector< std::vector<FluentTimeResource> > concurrentRequirements = FluentTimeResource::getMutualExclusive(mResourceRequirements);

        std::vector< std::vector<FluentTimeResource> >::const_iterator cit = concurrentRequirements.begin();
        for(; cit != concurrentRequirements.end(); ++cit)
        {
            LOG_DEBUG_S << "Concurrent requirements";
            const std::vector<FluentTimeResource>& concurrentFluents = *cit;
            for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
            {
                LOG_DEBUG_S << "    model: " << mAvailableModels[ mi ].toString();
                Gecode::IntVarArgs args;

                std::vector<FluentTimeResource>::const_iterator fit = concurrentFluents.begin();
                for(; fit != concurrentFluents.end(); ++fit)
                {
                    size_t fluentIdx = FluentTimeResource::getIndex(mResourceRequirements, *fit);
                    Gecode::IntVar v = resourceDistribution(mi,fluentIdx);
                    args << v;

                    LOG_DEBUG_S << "    index: " << mi << "/" << fluentIdx;
                }

                uint32_t maxCardinality = mModelPool[ mAvailableModels[mi] ];
                LOG_DEBUG_S << "Add general resource usage constraint: " << std::endl
                    << "     " << mAvailableModels[mi].toString() << "# <= " << maxCardinality;
                rel(*this, sum(args) <= maxCardinality);
            }
        }
    }
}

void TransportNetwork::initializeRoleDistributionConstraints()
{
    bool forceMinimumRoleUsage = mConfiguration.getValueAs<bool>("TransportNetwork/search/options/role-usage/force-min",false);
    bool mobileBoundedRoleUsage = mConfiguration.getValueAs<bool>("TransportNetwork/search/options/role-usage/mobile/bounded",false);
    int mobileRoleUsageBoundOffset = mConfiguration.getValueAs<int>("TransportNetwork/search/options/role-usage/mobile/bound-offset",false);
    bool immobileBoundedRoleUsage = mConfiguration.getValueAs<bool>("TransportNetwork/search/options/role-usage/immobile/bounded",false);
    int immobileRoleUsageBoundOffset = mConfiguration.getValueAs<int>("TransportNetwork/search/options/role-usage/immobile/bound-offset",false);

    // Role distribution
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());

    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());
    {
        Gecode::IntVarArgs mobileModelBounds;
        Gecode::IntVarArgs immobileModelBounds;
        // Sum of all models instances (role) has to correspond to the model count
        for(size_t modelIndex = 0; modelIndex < mAvailableModels.size(); ++modelIndex)
        {

            Gecode::IntVar mobileModelBound(*this,0, mobileRoleUsageBoundOffset);
            mobileModelBounds << mobileModelBound;
            Gecode::IntVar immobileModelBound(*this,0, immobileRoleUsageBoundOffset);
            immobileModelBounds << immobileModelBound;

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
                uint32_t maxCardinality = mModelPool[ mAvailableModels[modelIndex] ];
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
                    // This tries to bound the problem using the number of
                    // available instances
                    LOG_DEBUG_S << "Constraint general role usage: " << std::endl
                        << "     " << mAvailableModels[modelIndex].toString() << "# <= " << maxCardinality;
                    // Limit mobile / immobile systems
                    organization_model::facades::Robot robot(mAvailableModels[modelIndex], mAsk);
                    if(robot.isMobile())
                    {

                        if(mobileBoundedRoleUsage)
                        {
                            // This tries to bound the problem using the number of
                            // available instances
                            rel(*this, z >= modelCount);

                            LOG_DEBUG_S << "Constraint mobile role usage: " << std::endl
                                << "     " << mAvailableModels[modelIndex].toString() << "# <= " << modelCount << "+ " << mobileRoleUsageBoundOffset;
                            rel(*this, z <= (modelCount + mobileModelBound) );
                        } else {
                            rel(*this, z == modelCount);
                        }
                    }

                    if(!robot.isMobile())
                    {
                        if(immobileBoundedRoleUsage)
                        {
                            // This tries to bound the problem using the number of
                            // available instances
                            rel(*this, z >= modelCount);

                            LOG_DEBUG_S << "Constraint immobile role usage: " << std::endl
                                << "     " << mAvailableModels[modelIndex].toString() << "# <= " << modelCount << "+ " << immobileRoleUsageBoundOffset;
                            rel(*this, z <= (modelCount + immobileModelBound) );
                        } else {
                            rel(*this, z == modelCount);
                        }
                    }
                }
            }
        }
        if(!forceMinimumRoleUsage)
        {
            if(mobileBoundedRoleUsage)
            {
                rel(*this, sum(mobileModelBounds) <= mobileRoleUsageBoundOffset);
            }
            if(immobileBoundedRoleUsage)
            {
                rel(*this, sum(immobileModelBounds) <= immobileRoleUsageBoundOffset);
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

void TransportNetwork::enforceUnaryResourceUsage()
{
    // Role distribution
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    // Set of available models: mModelPool
    // Make sure the assignments are within resource bounds for concurrent requirements
    std::vector< std::vector<FluentTimeResource> > concurrentRequirements = FluentTimeResource::getMutualExclusive(mResourceRequirements);

    std::vector< std::vector<FluentTimeResource> >::const_iterator cit = concurrentRequirements.begin();
    if(!concurrentRequirements.empty())
    {
        for(; cit != concurrentRequirements.end(); ++cit)
        {
            const std::vector<FluentTimeResource>& concurrentFluents = *cit;
            for(size_t roleIndex = 0; roleIndex < mRoles.size(); ++roleIndex)
            {
                Gecode::IntVarArgs args;

                std::vector<FluentTimeResource>::const_iterator fit = concurrentFluents.begin();
                for(; fit != concurrentFluents.end(); ++fit)
                {
                    size_t row = FluentTimeResource::getIndex(mResourceRequirements, *fit);
                    LOG_DEBUG_S << "    index: " << roleIndex << "/" << row;
                    Gecode::IntVar v = roleDistribution(roleIndex, row);
                    args << v;
                }
                // there can only be one role active
                rel(*this, sum(args) <= 1);
            }
        }
    } else {
        LOG_DEBUG_S << "No concurrent requirements found";
    }
}

Gecode::Symmetries TransportNetwork::identifySymmetries()
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    Gecode::Symmetries symmetries;
    // define interchangeable columns for roles of the same model type
    owlapi::model::IRIList::const_iterator ait = mAvailableModels.begin();
    for(; ait != mAvailableModels.end(); ++ait)
    {
        const owlapi::model::IRI& currentModel = *ait;
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

TransportNetwork::TransportNetwork(bool share, TransportNetwork& other)
    : Gecode::Space(share, other)
    , mpMission(other.mpMission)
    , mModelPool(other.mModelPool)
    , mAsk(other.mAsk)
    , mServices(other.mServices)
    , mResources(other.mResources)
    , mIntervals(other.mIntervals)
    , mTimepoints(other.mTimepoints)
    , mLocations(other.mLocations)
    , mResourceRequirements(other.mResourceRequirements)
    , mAvailableModels(other.mAvailableModels)
    , mRoles(other.mRoles)
    , mActiveRoles(other.mActiveRoles)
    , mActiveRoleList(other.mActiveRoleList)
    , mSupplyDemand(other.mSupplyDemand)
    , mMinCostFlowSolution(other.mMinCostFlowSolution)
    , mMinCostFlowFlaws(other.mMinCostFlowFlaws)
    , mFlawResolution(other.mFlawResolution)
    , mConfiguration(other.mConfiguration)
    , mUseMasterSlave(other.mUseMasterSlave)
    , mpCurrentMaster(other.mpCurrentMaster)
    , mTemporalConstraintNetwork(other.mTemporalConstraintNetwork)
    , mpQualitativeTemporalConstraintNetwork(other.mpQualitativeTemporalConstraintNetwork)
{
    assert( mpMission->getOrganizationModel() );
    assert(!mIntervals.empty());
    mModelUsage.update(*this, share, other.mModelUsage);
    mRoleUsage.update(*this, share, other.mRoleUsage);
    mCost.update(*this, share, other.mCost);
    mNumberOfFlaws.update(*this, share, other.mNumberOfFlaws);
    mQualitativeTimepoints.update(*this, share, other.mQualitativeTimepoints);

    for(size_t i = 0; i < other.mTimelines.size(); ++i)
    {
        AdjacencyList array;
        mTimelines.push_back(array);
        mTimelines[i].update(*this, share, other.mTimelines[i]);
    }

    //mCapacities.update(*this, share, other.mCapacities);
}

Gecode::Space* TransportNetwork::copy(bool share)
{
    return new TransportNetwork(share, *this);
}

std::vector<TransportNetwork::Solution> TransportNetwork::solve(const templ::Mission::Ptr& mission, uint32_t minNumberOfSolutions, const Configuration& configuration)
{
    SolutionList solutions;
    mission->validateForPlanning();

    assert(mission->getOrganizationModel());
    assert(!mission->getTimeIntervals().empty());

    /// Allow to log the final results into a csv file
    CSVLogger csvLogger({"session",
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

    /// Check if interactive mode should be used during the solution process
    TransportNetwork::msInteractive = configuration.getValueAs<bool>("TransportNetwork/search/interactive",false);

    TransportNetwork* distribution = new TransportNetwork(mission, configuration);
    distribution->mUseMasterSlave = configuration.getValueAs<bool>("TransportNetwork/search/options/master-slave",false);

    {
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
            int stoptimeInMs = distribution->mConfiguration.getValueAs<int>("TransportNetwork/search/options/timeout_in_ms",600000);

            Gecode::Search::Options options;
            options.threads = threads;
            //Gecode::Search::Cutoff * c = Gecode::Search::Cutoff::constant(cutoff);
            //s*b^i, for i = 0,1,2,3,4
            Gecode::Search::Cutoff * c = Gecode::Search::Cutoff::geometric(cutoff,2);
            options.cutoff = c;
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
            options.stop = Gecode::Search::Stop::time(stoptimeInMs);

            //Gecode::BAB<TransportNetwork> searchEngine(distribution,options);
            //Gecode::DFS<TransportNetwork> searchEngine(distribution, options);
            Gecode::TemplRBS< TransportNetwork, Gecode::BAB > searchEngine(distribution, options);
            //Gecode::TemplRBS< TransportNetwork, Gecode::DFS > searchEngine(distribution, options);

        TransportNetwork* best = NULL;
        //try {
            int i = 0;
            while(TransportNetwork* current = searchEngine.next())
            {
                delete best;
                best = current;

                using namespace organization_model;

                LOG_INFO_S << "#" << i << "/" << minNumberOfSolutions << " solution found:" << current->toString();
                std::cout << "Solution found:" << std::endl;
                std::cout << "    # session id " << current->mpMission->getLogger()->getSessionId() << std::endl;
                std::cout << "    # flaws: " << current->mNumberOfFlaws.val() << std::endl;
                std::cout << "    # cost: " << current->mCost.val() << std::endl;
                std::cout << std::endl;

                csvLogger.addToRow(current->mpMission->getLogger()->getSessionId(),"session");
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

                std::string filename = mission->getLogger()->filename("search-statistics.csv");
                std::cout << "Saving stats in: " << filename << std::endl;
                csvLogger.save(filename);

                Solution solution = current->getSolution();

                try {
                    solvers::SolutionAnalysis sa(mission, solution.getMinCostFlowSolution());
                    sa.computePlan();
                    sa.quantifyTime();
                    sa.quantifyMetric();

                    //std::string tcnFilename = mission->getLogger()->filename("temporal-constraint-network.gexf");
                    //graph_analysis::io::GraphIO::write(tcnFilename, sa.getTimeDistanceGraph());
                } catch(const std::exception& e)
                {
                    LOG_WARN_S << e.what();
                }
                //sa.remainingFlaws();

                solutions.push_back(solution);
                // Saving solution
                saveSolution(solution, mission);

                if(minNumberOfSolutions != 0)
                {
                    if(solutions.size() == minNumberOfSolutions)
                    {
                        LOG_INFO_S << "Found minimum required number of solutions: " << solutions.size();
                        break;
                    }
                }
            }

            std::cout << "Solution Search" << std::endl;
            std::cout << "    was stopped (e.g. timeout): " << searchEngine.stopped() << std::endl;
            std::cout << "    found # solutions: " << solutions.size() << std::endl;
            std::cout << "    minimum request: " << minNumberOfSolutions << std::endl;
            std::cout << std::endl;
            std::cout << "Statistics: " << std::endl;
            std::cout << std::setw(15) << "    propagate " << searchEngine.statistics().propagate << std::endl;

            std::cout << std::setw(15) << "    fail " << searchEngine.statistics().fail << std::endl;
            std::cout << std::setw(15) << "    node " << searchEngine.statistics().node << std::endl;
            std::cout << std::setw(15) << "    depth " << searchEngine.statistics().depth << std::endl;
            std::cout << std::setw(15) << "    restart " << searchEngine.statistics().restart << std::endl;
            std::cout << std::setw(15) << "    nogood " << searchEngine.statistics().nogood << std::endl;
            csvLogger.addToRow(-1,"session");
            csvLogger.addToRow(searchEngine.stopped(), "solution-stopped");
            csvLogger.addToRow(searchEngine.statistics().propagate, "propagate");
            csvLogger.addToRow(searchEngine.statistics().fail, "fail");
            csvLogger.addToRow(searchEngine.statistics().node, "node");
            csvLogger.addToRow(searchEngine.statistics().depth, "depth");
            csvLogger.addToRow(searchEngine.statistics().restart, "restart");
            csvLogger.addToRow(searchEngine.statistics().nogood, "nogood");

            if(best)
            {
                std::cout << std::setw(15) << "    flaws " << best->mMinCostFlowFlaws.size() << std::endl;
                std::cout << std::setw(15) << "    cost " << best->cost().val() << std::endl;
                csvLogger.addToRow(1.0, "solution-found");
                csvLogger.addToRow(best->mMinCostFlowFlaws.size(), "flaws");
                csvLogger.addToRow(best->cost().val(), "cost");
            } else {
                std::cout << std::setw(15) << "    flaws " << "n/a" << std::endl;
                std::cout << std::setw(15) << "    cost " << "n/a" << std::endl;
                csvLogger.addToRow(0.0, "solution-found");
                csvLogger.addToRow(-1.0, "flaws");
                csvLogger.addToRow(-1.0, "cost");
            }
            csvLogger.commitRow();

        //} catch(const std::exception& e)
        //{
        //    std::cout << "Solution Search (failed) stopped: " << searchEngine.stopped() << std::endl;

        //    std::cout << "Statistics: " << std::endl;
        //    std::cout << std::setw(15) << "    propagate " << searchEngine.statistics().propagate << std::endl;

        //    std::cout << std::setw(15) << "    fail " << searchEngine.statistics().fail << std::endl;
        //    std::cout << std::setw(15) << "    node " << searchEngine.statistics().node << std::endl;
        //    std::cout << std::setw(15) << "    depth " << searchEngine.statistics().depth << std::endl;
        //    std::cout << std::setw(15) << "    restart " << searchEngine.statistics().restart << std::endl;
        //    std::cout << std::setw(15) << "    nogood " << searchEngine.statistics().nogood << std::endl;
        //    std::cout << std::setw(15) << "    flaws " << best->mMinCostFlowFlaws.size() << std::endl;
        //    std::cout << std::setw(15) << "    cost " << best->cost().val() << std::endl;
        //    csvLogger.addToRow(1.0, "solution-found");
        //    csvLogger.addToRow(searchEngine.stopped(), "solution-stopped");
        //    csvLogger.addToRow(searchEngine.statistics().propagate, "propagate");
        //    csvLogger.addToRow(searchEngine.statistics().fail, "fail");
        //    csvLogger.addToRow(searchEngine.statistics().node, "node");
        //    csvLogger.addToRow(searchEngine.statistics().depth, "depth");
        //    csvLogger.addToRow(searchEngine.statistics().restart, "restart");
        //    csvLogger.addToRow(searchEngine.statistics().nogood, "nogood");
        //    csvLogger.addToRow(-1, "flaws");
        //    csvLogger.addToRow(-1, "cost");
        //    csvLogger.commitRow();

        //    throw;
        //}
    }

    std::string filename = mission->getLogger()->filename("search-statistics.csv");
    LOG_INFO_S << "Saving stats in: " << filename;
    csvLogger.save(filename);

    delete distribution;
    return solutions;
}

solvers::Session::Ptr TransportNetwork::run(const templ::Mission::Ptr& mission, uint32_t minNumberOfSolutions, const Configuration& configuration)
{
    Session::Ptr session = make_shared<Session>(mission);
    SolutionList solutionList = TransportNetwork::solve(mission, minNumberOfSolutions, configuration);

    solvers::Solution::List solutions;
    session->setSolutions(solutions);
    return session;
}

void TransportNetwork::addConstraint(const Constraint::Ptr& constraint)
{
    mConstraints.push_back(constraint);

    MissionConstraintManager::apply(constraint, *this);
}

Constraint::PtrList TransportNetwork::getAssignmentsAsConstraints() const
{
    using namespace organization_model;
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

void TransportNetwork::appendToTupleSet(Gecode::TupleSet& tupleSet, const organization_model::ModelPool::Set& combinations) const
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
    organization_model::ModelPool::const_iterator cit = mModelPool.find(getResourceModelFromIndex(index));
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
    LOG_INFO_S << "Model usage: " << modelUsageToString();
    LOG_INFO_S << "Role usage: " << roleUsageToString();

    return activeRoles;
}

organization_model::ModelPool TransportNetwork::currentMinModelAssignment(const FluentTimeResource& ftr) const
{
    organization_model::ModelPool modelPool;
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
    // Update the timepoint comparator
    for(temporal::Interval& i : mIntervals)
    {
        i.setTimePointComparator(tcp);
    }

    // update timepoint comparator for intervals

    // General resource constraints
    //  - identify overlapping fts, limit resources for these (TODO: better
    //    identification of overlapping requirements)
    //
    // TODO: in brancher since its time dependant
    setUpperBoundForConcurrentRequirements();

    // There can be only one assignment per role
    //
    // TODO: in brancher since its time dependant
    enforceUnaryResourceUsage();
}

void TransportNetwork::doPostMinMaxConstraints(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).initializeMinMaxConstraints();
}

void TransportNetwork::doPostExtensionalContraints(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).addExtensionalConstraints();
}

void TransportNetwork::doPostRoleAssignments(Gecode::Space& home)
{
    if(msInteractive)
    {
        std::cout << "doPostRoleAssignments()" << std::endl;
        std::cout << "Press ENTER to continue..."  << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }
    static_cast<TransportNetwork&>(home).postRoleAssignments();
}

void TransportNetwork::postRoleAssignments()
{
    (void) status();

    LOG_WARN_S << "Posting Role Assignments: request status" << std::endl
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

    Role::List activeRoles;

    LOG_WARN_S << std::endl
        << mTimepoints << std::endl
        << symbols::constants::Location::toString(mLocations);

    assert(!mActiveRoles.empty());
    std::vector<uint32_t>::const_iterator rit = mActiveRoles.begin();

    assert(mTimelines.empty());

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
        Gecode::SetVarArray timeline(*this, locationTimeSize, Gecode::IntSet::empty, Gecode::IntSet(0,locationTimeSize-1),0,1);
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
                    LOG_DEBUG_S << "UPDATE column: view " << v << col;
                    v.intersect(*this, col,col);
                    v.cardMin(*this, 1);
                    v.cardMax(*this, 1);
                    LOG_DEBUG_S << "Result view " << v << col;

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
    LOG_INFO_S << "Using active roles: " << Role::toString(activeRoles);

    LOG_INFO_S << "Timelines after first propagation of requirements: " << std::endl
        << Formatter::toString(mTimelines, numberOfFluents);

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
            organization_model::facades::Robot robot(role.getModel(), mAsk);
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
    for(size_t i = 0; i < mActiveRoles.size(); ++i)
    {
        propagators::isPath(*this, mTimelines[i], mActiveRoleList[i].toString(), numberOfTimepoints, mLocations.size());

        // Only branch on the mobile systems
        organization_model::facades::Robot robot(mActiveRoleList[i].getModel(), mAsk);
        if(robot.isMobile())
        {
            Gecode::SetAFC timelineUsageAfc(*this, mTimelines[i], timelineAfcDecay);
            // SET_VAR_MERIT MIN
            branch(*this, mTimelines[i],Gecode::SET_VAR_AFC_MIN(timelineAfcDecay), Gecode::SET_VAL_RND_EXC(rnd));
            //branch(*this, mTimelines[i],Gecode::SET_VAR_RND(rnd),Gecode::SET_VAL_RND_EXC(rnd));
        }
    }

    // Only the check whether a feasible approach is to use a heuristic
    // to draw system by supply demand
    //branchTimelines(*this, mTimelines, mSupplyDemand);
    Gecode::branch(*this, &TransportNetwork::doPostMinCostFlow);

//    Gecode::Gist::stopBranch(*this);
}


void TransportNetwork::doPostMinCostFlow(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).postMinCostFlow();
}

void TransportNetwork::postMinCostFlow()
{
    if(msInteractive)
    {
        std::cout << "postMinCostFlow" << std::endl;
        std::cout << "Press ENTER to proceed ..." << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }

    LOG_INFO_S << "Post MinCostFlow";
    save();

    std::map<Role, csp::RoleTimeline> minimalTimelines =  RoleTimeline::computeTimelines(*mpMission.get(), getRoleDistribution());

    LOG_DEBUG_S << "RoleTimelines: " << std::endl
        << RoleTimeline::toString(minimalTimelines, 4);

    //assert(!timelines.empty());
    try {
        if(msInteractive)
        {
            std::cout << "Remaining flaws computation: " << mMinCostFlowFlaws.size() << std::endl;
            std::cout << "     cost: " << mCost << std::endl;
            std::cout << "     flaws: " << mNumberOfFlaws << std::endl;
            std::cout << "Press ENTER to continue..." << std::endl;
            std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }

        SpaceTime::Timelines spaceTimeTimelines = getTimelines();
        std::pair<SpaceTime::Timelines, std::map<Role, csp::RoleTimeline> > key(spaceTimeTimelines, minimalTimelines);
        transshipment::MinCostFlow minCostFlow(mpMission, minimalTimelines, spaceTimeTimelines);

        FlowSolutions::iterator it = msMinCostFlowSolutions.find(key);
        if(it == msMinCostFlowSolutions.end())
        {
            std::vector<transshipment::Flaw> flaws = minCostFlow.run();

            //std::cout << "Found " << flaws.size() << " flaws";
            //LOG_INFO_S << "Found " << flaws.size() << " flaws";
            //for(const transshipment::Flaw& flaw: flaws)
            //{
            //    LOG_INFO_S << "Flaw: " << flaw.toString();
            //}

            transshipment::FlowNetwork flowNetwork = minCostFlow.getFlowNetwork();
            mMinCostFlowSolution = flowNetwork.getSpaceTimeNetwork();
            flowNetwork.save();

            // store all flaws
            mMinCostFlowFlaws = flaws;

            msMinCostFlowSolutions[key] = FlowSolutionValue(flaws, mMinCostFlowSolution);
        } else {
            if(msInteractive)
            {
                std::cout << "Found existing solution .. (skipping recomputation and taking from cache)" << std::endl;
                std::cout << "Press ENTER to continue..." << std::endl;
                std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
            }

            mMinCostFlowFlaws = it->second.first;
            mMinCostFlowSolution = it->second.second;
        }
        // compute all feasible resolution that might allow
        // to improve the solution
        mFlawResolution.prepare(mMinCostFlowFlaws);

        std::cout << "Remaining flaws: " << mMinCostFlowFlaws.size() << std::endl;
        if(msInteractive)
        {
            std::cout << "Remaining flaws: " << mMinCostFlowFlaws.size() << std::endl;
            std::cout << "Press ENTER to continue..." << std::endl;
            std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }

        //if(flaws.empty())
        //{
        //    // found a zero flaw solution
        //    // Generate constraints to solve this issue
        //} else {
        ////    this->fail();
        //}
        // Set flaws as current cost of this solution
        rel(*this, mCost, Gecode::IRT_EQ, mMinCostFlowFlaws.size());
        // Set flaws as well
        rel(*this, mNumberOfFlaws, Gecode::IRT_EQ, mMinCostFlowFlaws.size());

    } catch(const std::runtime_error& e)
    {
        // Min cost flow optimization
        std::cout << "Could not find solution: " << e.what() << std::endl;
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
    if(msInteractive)
    {
        std::cout << "Post timelines" << std::endl;
        std::cout << "Press ENTER to continue ..." << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }
    branchTimelines(*this, mTimelines, mSupplyDemand);
}

std::string TransportNetwork::toString() const
{
    std::stringstream ss;
    ss << "TransportNetwork: #" << std::endl;
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
    for(size_t m = 0; m < mRoles.size(); ++m)
    {
        ss << std::setw(30) << mRoles[m].toString() << ": ";
        for(size_t i = 0; i < mResourceRequirements.size(); ++i)
        {
            ss << std::setw(10) << std::left << rolesDistribution(m,i);
        }
        ss << std::endl;
    }
    //ss << "Current model usage: " << std::endl << resourceDistribution << std::endl;
    //ss << "Current role usage: " << std::endl << rolesDistribution << std::endl;

    try {
        for(size_t i = 0; i < mTimelines.size(); ++i)
        {
            ss << mActiveRoleList[i].toString();
            ss << Formatter::toString(mTimelines[i], mLocations.size()) << std::endl;
        }

        SpaceTime::Timelines timelines = TypeConversion::toTimelines(mActiveRoleList, mTimelines, mLocations, mTimepoints);
        ss << "Number of timelines: " << timelines.size() << " active roles -- " << mActiveRoleList.size() << std::endl;
        //ss << "Current timelines:" << std::endl << toString(timelines) << std::endl;
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
    std::stringstream ss;
    ss << "Model usage:" << std::endl;
    ss << std::setw(30) << std::right << "    FluentTimeResource: ";
    for(size_t r = 0; r < mResourceRequirements.size(); ++r)
    {
        const FluentTimeResource& fts = mResourceRequirements[r];
        /// construct string for proper alignment
        std::string s = fts.getFluent()->getInstanceName();
        s += "@[" + fts.getInterval().toString(0,true) + "]";

        ss << std::setw(15) << std::left << s;
    }
    ss << std::endl;

    int modelIndex = 0;
    organization_model::ModelPool::const_iterator cit = mModelPool.begin();
    for(; cit != mModelPool.end(); ++cit, ++modelIndex)
    {
        const owlapi::model::IRI& model = cit->first;
        ss << std::setw(30) << std::left << model.getFragment() << ": ";
        for(size_t r = 0; r < mResourceRequirements.size(); ++r)
        {
            ss << std::setw(15) << mModelUsage[r*mModelPool.size() + modelIndex] << " ";
        }
        ss << std::endl;
    }
    return ss.str();
}

std::string TransportNetwork::roleUsageToString() const
{
    std::stringstream ss;
    ss << "Role usage:" << std::endl;
    ss << std::setw(30) << std::right << "    FluentTimeResource: ";
    for(size_t r = 0; r < mResourceRequirements.size(); ++r)
    {
        const FluentTimeResource& fts = mResourceRequirements[r];
        /// construct string for proper alignment
        std::string s = fts.getFluent()->getInstanceName();
        s += "@[" + fts.getInterval().toString(0,true) + "]";

        ss << std::setw(15) << std::left << s;
    }
    ss << std::endl;

    for(size_t i = 0; i < mRoles.size(); ++i)
    {
        ss << std::setw(30) << std::left << mRoles[i].toString() << ": ";
        for(size_t r = 0; r < mResourceRequirements.size(); ++r)
        {
            ss << std::setw(15) << mRoleUsage[r*mRoles.size() + i] << " ";
        }
        ss << std::endl;
    }
    return ss.str();
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

} // end namespace csp
} // end namespace solvers
} // end namespace templ
