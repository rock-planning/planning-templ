#include "TransportNetwork.hpp"
#include <base-logging/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include <gecode/minimodel.hh>
#include <gecode/set.hh>
#include <gecode/gist.hh>
#include <gecode/search.hh>

#include <iomanip>
#include <Eigen/Dense>

#include <organization_model/Algebra.hpp>
#include <organization_model/vocabularies/OM.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/symbols/object_variables/LocationCardinality.hpp>
#include <templ/SpaceTime.hpp>
#include <organization_model/facets/Robot.hpp>

#include "ConstraintMatrix.hpp"
#include "propagators/IsPath.hpp"
#include "propagators/IsValidTransportEdge.hpp"
#include "propagators/MultiCommodityFlow.hpp"
#include "utils/Formatter.hpp"

using namespace templ::solvers::csp::utils;

namespace templ {
namespace solvers {
namespace csp {

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

TransportNetwork::Solution TransportNetwork::getSolution() const
{
    Solution solution;
    solution.mModelDistribution = getModelDistribution();
    solution.mRoleDistribution = getRoleDistribution();
    solution.mTimelines = getTimelines();
    return solution;
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
    return TypeConversion::toTimelines(mActiveRoleList, mTimelines, mLocations, mTimepoints);
}

std::set< std::vector<uint32_t> > TransportNetwork::toCSP(const organization_model::ModelPoolSet& combinations) const
{
    std::set< std::vector<uint32_t> > csp_combinations;
    organization_model::ModelPoolSet::const_iterator cit = combinations.begin();
    for(; cit != combinations.end(); ++cit)
    {
        csp_combinations.insert( toCSP(*cit) );
    }
    return csp_combinations;
}

std::vector<uint32_t> TransportNetwork::toCSP(const organization_model::ModelPool& combination) const
{
    // return index of model and count per model
    std::vector<uint32_t> csp_combination(mModelPool.size(),0);

    organization_model::ModelPool::const_iterator cit = combination.begin();
    for(; cit != combination.end(); ++cit)
    {
        uint32_t index = systemModelToCSP(cit->first);
        csp_combination.at(index) = cit->second;
    }
    return csp_combination;
}

uint32_t TransportNetwork::systemModelToCSP(const owlapi::model::IRI& model) const
{
    owlapi::model::IRIList::const_iterator cit = std::find(mAvailableModels.begin(),
            mAvailableModels.end(), model);

    if(cit == mAvailableModels.end())
    {
        throw std::invalid_argument("templ::solvers::csp::TransportNetwork::systemModelToCSP:"
                " unknown model '" + model.toString() );
    } else {
        return (cit - mAvailableModels.begin());
    }

}

TransportNetwork::TransportNetwork(const templ::Mission::Ptr& mission)
    : Gecode::Space()
    , mpMission(mission)
    , mModelPool(mpMission->getAvailableResources())
    , mAsk(mpMission->getOrganizationModel(), mpMission->getAvailableResources(), true)
    , mResources(mpMission->getRequestedResources())
    , mIntervals(mpMission->getTimeIntervals())
    , mTimepoints(mpMission->getOrderedTimepoints())
    , mLocations(mpMission->getLocations())
    , mResourceRequirements(getResourceRequirements())
    , mModelUsage(*this, /*# of models*/ mpMission->getAvailableResources().size()*
            /*# of fluent time services*/mResourceRequirements.size(), 0, getMaxResourceCount(mModelPool)) // maximum number of model at that point
    , mAvailableModels(mpMission->getModels())
    , mRoleUsage(*this, /*width --> col */ mission->getRoles().size()* /*height --> row*/ mResourceRequirements.size(), 0, 1) // Domain 0,1 to represent activation
    , mRoles(mission->getRoles())
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

    //Gecode::IntArgs x = Gecode::IntArgs::create(3,2,0);
    //Gecode::IntArgs y = Gecode::IntArgs::create(2,2,0);
    //Gecode::IntArgs z = Gecode::IntArgs::create(2,2,0);
    //LOG_DEBUG_S << "Intargs: " << x;
    //LOG_DEBUG_S << "Intargs: " << Gecode::IntSet( x );

    ////rel(*this, x + y, Gecode::SRT_SUB, z)

    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());
    // Example usage
    //
    //Gecode::IntVar v = resourceDistribution(0,0);
    //
    // Maximum upper bound
    //Gecode::IntVar v0 = resourceDistribution(0,1);
    //Gecode::IntVarArgs a;
    //a << v;
    //a << v0;

    // Bounding individual model count
    //rel(*this, v, Gecode::IRT_GQ, 0);
    //rel(*this, v, Gecode::IRT_LQ, 1);
    //rel(*this, sum(a) < 1);

    // Add explit constraints, i.e. set of model combinations can be added this
    // way
    // Gecode::TupleSet tupleSet;
    // tupleSet.add( Gecode::IntArgs::create( mission.getAvailableResources().size(), 1, 0) );
    // tupleSet.finalize();
    //
    // extensional(*this, resourceDistribution.row(0), tupleSet);

    // Outline:
    // (A) for each requirement add the min/max and existential constraints
    // for all overlapping requirements create maximum resource constraints
    initializeMinMaxConstraints();

    // (B) General resource constraints
    //     - identify overlapping fts, limit resources for these (TODO: better
    //     indentification of overlapping requirements)
    //
    setUpperBoundForConcurrentRequirements();

    // Limit roles to resource availability
    initializeRoleDistributionConstraints();
    // There can be only one assignment per role
    enforceUnaryResourceUsage();

    // (C) Avoid computation of solutions that are redunant
    // Gecode documentation says however in 8.10.2 that "Symmetry breaking by
    // LDSB is not guaranteed to be complete. That is, a search may still return
    // two distinct solutions that are symmetric."
    //
    Gecode::Symmetries symmetries = identifySymmetries();

    // Regarding the use of INT_VALUES_MIN() and INT_VALUES_MAX(): "This is
    // typically a poor choice, as none of the alternatives can benefit from
    // propagation that arises when other values of the same variable are tried.
    // These branchings exist for instructional purposes" p.123 Tip 8.2
    //
    //branch(*this, mRoleUsage, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_MIN(), symmetries);
    //branch(*this, mRoleUsage, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_MIN(), symmetries);
    //branch(*this, mRoleUsage, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_MIN(), symmetries);
    //branch(*this, mModelUsage, Gecode::INT_VAR_AFC_MIN(afc), Gecode::INT_VAL_MIN());

    Gecode::IntAFC modelUsageAfc(*this, mModelUsage, 0.99);
    modelUsageAfc.decay(*this, 0.95);
    branch(*this, mModelUsage, Gecode::INT_VAR_AFC_MIN(modelUsageAfc), Gecode::INT_VAL_SPLIT_MIN());

    Gecode::Rnd modelUsageRnd(1U);
    branch(*this, mModelUsage, Gecode::INT_VAR_AFC_MIN(modelUsageAfc), Gecode::INT_VAL_RND(modelUsageRnd));



    Gecode::IntAFC roleUsageAfc(*this, mRoleUsage, 0.99);
    roleUsageAfc.decay(*this, 0.95);
    branch(*this, mRoleUsage, Gecode::INT_VAR_AFC_MIN(roleUsageAfc), Gecode::INT_VAL_SPLIT_MIN());

    Gecode::Rnd roleUsageRnd(1U);
    branch(*this, mRoleUsage, Gecode::INT_VAR_AFC_MIN(roleUsageAfc), Gecode::INT_VAL_RND(roleUsageRnd));

    Gecode::Gist::stopBranch(*this);
    // see 8.14 Executing code between branchers
    Gecode::branch(*this, &TransportNetwork::assignRoles);

    Gecode::Gist::Print<TransportNetwork> p("Print solution");
    Gecode::Gist::Options o;
    o.inspect.click(&p);
    Gecode::Gist::bab(this, o);

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
                        // default min requirement is 0
                        uint32_t minCardinality = 0;
                        /// Consider resource cardinality constraint
                        LOG_DEBUG_S << "Check min cardinality for " << mAvailableModels[mi];
                        organization_model::ModelPool::const_iterator cardinalityIt = fts.minCardinalities.find( mAvailableModels[mi] );
                        if(cardinalityIt != fts.minCardinalities.end())
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

                // Prepare the extensional constraints, i.e. specifying the allowed
                // combinations
                organization_model::ModelPoolSet allowedCombinations = fts.getDomain();
                appendToTupleSet(mExtensionalConstraints[requirementIndex], allowedCombinations);

                // there can be no empty assignment for resource requirement
                rel(*this, sum( resourceDistribution.row(requirementIndex) ) > 0);

                // This can be equivalently modelled using a linear constraint
                // Gecode::IntArgs c(mAvailableModels.size());
                // for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
                //    c[mi] = 1;
                // linear(*this, c, resourceDistribution.row(requirementIndex), Gecode::IRT_GR, 0);
            }
        }

        // use the prepared list of extensional constraints to activate the
        // constraints
        for(auto pair : mExtensionalConstraints)
        {
            uint32_t requirementIndex = pair.first;
            Gecode::TupleSet& tupleSet = pair.second;

            tupleSet.finalize();
            extensional(*this, resourceDistribution.row(requirementIndex), tupleSet);
        }

        LOG_INFO_S << constraintMatrix.toString();
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
        std::vector< std::vector<FluentTimeResource> > concurrentRequirements = FluentTimeResource::getConcurrent(mResourceRequirements, mIntervals);

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
                    Gecode::IntVar v = resourceDistribution(mi, getFluentIndex(*fit));
                    args << v;

                    LOG_DEBUG_S << "    index: " << mi << "/" << getFluentIndex(*fit);
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
    // Role distribution
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mpMission->getAvailableResources().size(), /*height --> row*/ mResourceRequirements.size());

    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());
    {
        // Sum of all role instances has to correspond to the model count
        for(size_t modelIndex = 0; modelIndex < mAvailableModels.size(); ++modelIndex)
        {
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
                rel(*this, sum(args) == modelCount);
            }
        }
    }
}

void TransportNetwork::enforceUnaryResourceUsage()
{
    // Role distribution
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    // Set of available models: mModelPool
    // Make sure the assignments are within resource bounds for concurrent requirements
    std::vector< std::vector<FluentTimeResource> > concurrentRequirements = FluentTimeResource::getConcurrent(mResourceRequirements, mIntervals);

    std::vector< std::vector<FluentTimeResource> >::const_iterator cit = concurrentRequirements.begin();
    if(!concurrentRequirements.empty())
    {
        for(; cit != concurrentRequirements.end(); ++cit)
        {
            LOG_DEBUG_S << "Concurrent roles requirements: " << mRoles.size();
            const std::vector<FluentTimeResource>& concurrentFluents = *cit;

            for(size_t roleIndex = 0; roleIndex < mRoles.size(); ++roleIndex)
            {
                Gecode::IntVarArgs args;

                std::vector<FluentTimeResource>::const_iterator fit = concurrentFluents.begin();
                for(; fit != concurrentFluents.end(); ++fit)
                {
                    size_t row = getFluentIndex(*fit);
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
    , mIntervals(other.mIntervals)
    , mTimepoints(other.mTimepoints)
    , mLocations(other.mLocations)
    , mResourceRequirements(other.mResourceRequirements)
    , mAvailableModels(other.mAvailableModels)
    , mRoles(other.mRoles)
    , mActiveRoles(other.mActiveRoles)
    , mActiveRoleList(other.mActiveRoleList)
{
    assert( mpMission->getOrganizationModel() );
    assert(!mIntervals.empty());
    mModelUsage.update(*this, share, other.mModelUsage);
    mRoleUsage.update(*this, share, other.mRoleUsage);

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

std::vector<TransportNetwork::Solution> TransportNetwork::solve(const templ::Mission::Ptr& mission, uint32_t minNumberOfSolutions)
{
    SolutionList solutions;
    mission->validateForPlanning();

    assert(mission->getOrganizationModel());
    assert(!mission->getTimeIntervals().empty());

    TransportNetwork* distribution = new TransportNetwork(mission);
    {
            Gecode::Search::Options options;
            options.threads = 1;
            Gecode::Search::Cutoff * c = Gecode::Search::Cutoff::constant(70000);
            options.cutoff = c;

            //Gecode::BAB<TransportNetwork> searchEngine(distribution,options);
            //Gecode::DFS<TransportNetwork> searchEngine(this);
            Gecode::RBS< TransportNetwork, Gecode::BAB > searchEngine(distribution, options);

        try {
            TransportNetwork* best = NULL;
            while(TransportNetwork* current = searchEngine.next())
            {
                delete best;
                best = current;

                using namespace organization_model;

                LOG_INFO_S << "Solution found:" << current->toString();
                solutions.push_back(current->getSolution());
                if(minNumberOfSolutions != 0)
                {
                    if(solutions.size() == minNumberOfSolutions)
                    {
                        LOG_INFO_S << "Found minimum required number of solutions: " << solutions.size();
                        break;
                    }
                }
            }

            if(best == NULL)
            {
                delete distribution;
                throw std::runtime_error("templ::solvers::csp::TransportNetwork::solve: no solution found");
            }
        //    delete best;
            best = NULL;

            std::cout << "Statistics: " << std::endl;
            std::cout << "    propagate " << searchEngine.statistics().propagate << std::endl;

            std::cout << "    fail " << searchEngine.statistics().fail << std::endl;
            std::cout << "    node " << searchEngine.statistics().node << std::endl;
            std::cout << "    depth" << searchEngine.statistics().depth << std::endl;
            std::cout << "    restart " << searchEngine.statistics().restart << std::endl;
            std::cout << "    nogoods " << searchEngine.statistics().nogood << std::endl;
        } catch(const std::exception& e)
        {
            std::cout << "Statistics: " << std::endl;
            std::cout << "    propagate " << searchEngine.statistics().propagate << std::endl;

            std::cout << "    fail " << searchEngine.statistics().fail << std::endl;
            std::cout << "    node " << searchEngine.statistics().node << std::endl;
            std::cout << "    depth" << searchEngine.statistics().depth << std::endl;
            std::cout << "    restart " << searchEngine.statistics().restart << std::endl;
            std::cout << "    nogoods " << searchEngine.statistics().nogood << std::endl;

            throw;
        }
    }

    delete distribution;
    return solutions;
}

void TransportNetwork::appendToTupleSet(Gecode::TupleSet& tupleSet, const organization_model::ModelPoolSet& combinations) const
{
    std::set< std::vector<uint32_t> > csp = toCSP( combinations );
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

size_t TransportNetwork::getFluentIndex(const FluentTimeResource& fluent) const
{
    std::vector<FluentTimeResource>::const_iterator ftsIt = std::find(mResourceRequirements.begin(), mResourceRequirements.end(), fluent);
    if(ftsIt != mResourceRequirements.end())
    {
        int index = ftsIt - mResourceRequirements.begin();
        assert(index >= 0);
        return (size_t) index;
    }

    throw std::runtime_error("templ::solvers::csp::TransportNetwork::getFluentIndex: could not find fluent index for '" + fluent.toString() + "'");
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

FluentTimeResource TransportNetwork::fromLocationCardinality(const temporal::PersistenceCondition::Ptr& p) const
{
    using namespace templ::solvers::temporal;
    point_algebra::TimePointComparator timepointComparator(mpMission->getTemporalConstraintNetwork());

    const symbols::StateVariable& stateVariable = p->getStateVariable();
    owlapi::model::IRI resourceModel(stateVariable.getResource());
    symbols::ObjectVariable::Ptr objectVariable = dynamic_pointer_cast<symbols::ObjectVariable>(p->getValue());
    symbols::object_variables::LocationCardinality::Ptr locationCardinality = dynamic_pointer_cast<symbols::object_variables::LocationCardinality>(objectVariable);

    Interval interval(p->getFromTimePoint(), p->getToTimePoint(), timepointComparator);
    std::vector<Interval>::const_iterator iit = std::find(mIntervals.begin(), mIntervals.end(), interval);
    if(iit == mIntervals.end())
    {
        LOG_INFO_S << "Size of intervals: " << mIntervals.size();
        throw std::runtime_error("templ::solvers::csp::TransportNetwork::getResourceRequirements: could not find interval: '" + interval.toString() + "'");
    }

    owlapi::model::IRIList::const_iterator sit = std::find(mResources.begin(), mResources.end(), resourceModel);
    if(sit == mResources.end())
    {
        throw std::runtime_error("templ::solvers::csp::TransportNetwork::getResourceRequirements: could not find service: '" + resourceModel.toString() + "'");
    }

    symbols::constants::Location::Ptr location = locationCardinality->getLocation();
    std::vector<symbols::constants::Location::Ptr>::const_iterator lit = std::find(mLocations.begin(), mLocations.end(), location);
    if(lit == mLocations.end())
    {
        throw std::runtime_error("templ::solvers::csp::TransportNetwork::getResourceRequirements: could not find location: '" + location->toString() + "'");
    }

    // Map objects to numeric indices -- the indices can be mapped
    // backed using the mission they were created from
    uint32_t timeIndex = iit - mIntervals.begin();
    FluentTimeResource ftr(mpMission,
            (int) (sit - mResources.begin())
            , timeIndex
            , (int) (lit - mLocations.begin()));

    if(mAsk.ontology().isSubClassOf(resourceModel, organization_model::vocabulary::OM::Functionality()))
    {
        // retrieve upper bound
        ftr.maxCardinalities = mAsk.getFunctionalSaturationBound(resourceModel);

    } else if(mAsk.ontology().isSubClassOf(resourceModel, organization_model::vocabulary::OM::Actor()))
    {
        switch(locationCardinality->getCardinalityRestrictionType())
        {
            case owlapi::model::OWLCardinalityRestriction::MIN :
            {
                size_t min = ftr.minCardinalities.getValue(resourceModel, std::numeric_limits<size_t>::min());
                ftr.minCardinalities[ resourceModel ] = std::max(min, (size_t) locationCardinality->getCardinality());
                break;
            }
            case owlapi::model::OWLCardinalityRestriction::MAX :
            {
                size_t max = ftr.maxCardinalities.getValue(resourceModel, std::numeric_limits<size_t>::max());
                ftr.maxCardinalities[ resourceModel ] = std::min(max, (size_t) locationCardinality->getCardinality());
                break;
            }
            default:
                break;
        }
    } else {
        throw std::invalid_argument("Unsupported state variable: " + resourceModel.toString());
    }

    ftr.maxCardinalities = organization_model::Algebra::max(ftr.maxCardinalities, ftr.minCardinalities);
    return ftr;
}

std::vector<FluentTimeResource> TransportNetwork::getResourceRequirements() const
{
    if(mIntervals.empty())
    {
        throw std::runtime_error("solvers::csp::TransportNetwork::getResourceRequirements: no time intervals available"
                " -- make sure you called prepareTimeIntervals() on the mission instance");
    }

    std::vector<FluentTimeResource> requirements;

    // Iterate over all existing persistence conditions
    // -- pick the ones relating to location-cardinality function
    using namespace templ::solvers::temporal;
    const std::vector<PersistenceCondition::Ptr>& conditions = mpMission->getPersistenceConditions();
    std::vector<PersistenceCondition::Ptr>::const_iterator cit = conditions.begin();
    for(; cit != conditions.end(); ++cit)
    {
        PersistenceCondition::Ptr p = *cit;

        symbols::StateVariable stateVariable = p->getStateVariable();
        if(stateVariable.getFunction() == symbols::ObjectVariable::TypeTxt[symbols::ObjectVariable::LOCATION_CARDINALITY] )
        {
            try {
                FluentTimeResource ftr = fromLocationCardinality( p );
                requirements.push_back(ftr);
                LOG_DEBUG_S << ftr.toString();
            } catch(const std::invalid_argument& e)
            {
                LOG_WARN_S << e.what();
            }
        }
    }

    // If multiple requirement exists that have the same interval
    // they can be compacted into one requirement
    compact(requirements);

    // Sort the requirements based on the start timepoint, i.e. the from
    using namespace templ::solvers::temporal;
    point_algebra::TimePointComparator timepointComparator(mpMission->getTemporalConstraintNetwork());
    std::sort(requirements.begin(), requirements.end(), [&timepointComparator](const FluentTimeResource& a,const FluentTimeResource& b) -> bool
            {
                return timepointComparator.lessThan(a.getInterval().getFrom(), b.getInterval().getFrom());
            });
    return requirements;
}

void TransportNetwork::compact(std::vector<FluentTimeResource>& requirements) const
{
    LOG_DEBUG_S << "BEGIN compact requirements";
    std::vector<FluentTimeResource>::iterator it = requirements.begin();
    for(; it != requirements.end(); ++it)
    {
        FluentTimeResource& fts = *it;

        std::vector<FluentTimeResource>::iterator compareIt = it + 1;
        for(; compareIt != requirements.end();)
        {
            FluentTimeResource& otherFts = *compareIt;

            if(fts.time == otherFts.time && fts.fluent == otherFts.fluent)
            {
                LOG_DEBUG_S << "Compacting: " << std::endl
                    << fts.toString() << std::endl
                    << otherFts.toString() << std::endl;

                // Compacting the resource list
                fts.resources.insert(otherFts.resources.begin(), otherFts.resources.end());

                // Use the functional saturation bound on all functionalities
                // after compacting the resource list
                std::set<organization_model::Functionality> functionalities;
                std::set<uint32_t>::const_iterator cit = fts.resources.begin();
                for(; cit != fts.resources.end(); ++cit)
                {
                    const owlapi::model::IRI& resourceModel = mResources[*cit];
                    using namespace owlapi;

                    if( mAsk.ontology().isSubClassOf(resourceModel, organization_model::vocabulary::OM::Functionality()))
                    {
                        organization_model::Functionality functionality(resourceModel);
                        functionalities.insert(functionality);
                    }
                }
                fts.maxCardinalities = mAsk.getFunctionalSaturationBound(functionalities);

                // MaxMin --> min cardinalities are a lower bound specified explicitely
                LOG_DEBUG_S << "Update Requirements: min: " << fts.minCardinalities.toString();
                LOG_DEBUG_S << "Update Requirements: otherMin: " << otherFts.minCardinalities.toString();

                fts.minCardinalities = organization_model::Algebra::max(fts.minCardinalities, otherFts.minCardinalities);
                LOG_DEBUG_S << "Result min: " << fts.minCardinalities.toString();

                // Resource constraints might enforce a minimum cardinality that is higher than the functional saturation bound
                // thus update the max cardinalities
                fts.maxCardinalities = organization_model::Algebra::max(fts.minCardinalities, fts.maxCardinalities);

                LOG_DEBUG_S << "Update requirement: " << fts.toString();

                requirements.erase(compareIt);
            } else {
                ++compareIt;
            }
        }
    }
    LOG_DEBUG_S << "END compact requirements";
}

bool TransportNetwork::isRoleForModel(uint32_t roleIndex, uint32_t modelIndex) const
{
    return mRoles.at(roleIndex).getModel() == mAvailableModels.at(modelIndex);
}

void TransportNetwork::assignRoles(Gecode::Space& home)
{
    LOG_WARN_S << "Assign Roles";
    static_cast<TransportNetwork&>(home).postRoleAssignments();
}

std::vector<uint32_t> TransportNetwork::getActiveRoles() const
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
    LOG_WARN_S << "Model usage: " << modelUsageToString();
    LOG_WARN_S << "Role usage: " << roleUsageToString();

    return activeRoles;
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
    // -- add an additional transfer location to allow for 'timegaps'
    mLocations.push_back(symbols::constants::Location::Ptr(new symbols::constants::Location("in-transfer")));

    size_t numberOfFluents = mLocations.size();
    size_t numberOfTimepoints = mTimepoints.size();
    size_t locationTimeSize = numberOfFluents*numberOfTimepoints;

    mActiveRoles = getActiveRoles();

    Role::List activeRoles;

    LOG_WARN_S << std::endl
        << mTimepoints << std::endl
        << symbols::constants::Location::toString(mLocations);

    assert(!mActiveRoles.empty());
    std::vector<uint32_t>::const_iterator rit = mActiveRoles.begin();
    for(; rit != mActiveRoles.end(); ++rit)
    {
        uint32_t roleIndex = *rit;
        const Role& role = mRoles[roleIndex];
        LOG_WARN_S << "Using active role: " << role.toString();

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

        // Setup the basic contraints for the timeline
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

            // Limit to one outgoing edge per timestep
            // Less or equal cardinality of 1
            // sum of cardinalities
            Gecode::linear(*this, cardinalities, Gecode::IRT_LQ, 1);
        }
        propagators::isPath(*this, timeline, numberOfTimepoints, numberOfFluents);

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
                    int allowed = timeIndex*numberOfFluents + fts.fluent;

                    // index of the location is fts.fluent
                    // edge index:
                    // row = timepointIdx*#ofLocations + from-location-offset
                    // col = (timepointIdx + 1) *#ofLocations + to-location-offset
                    //
                    // location (offset) = row % #ofLocations
                    // timepointIndex = (row - location(offset)) / #ofLocations
                    size_t row = FluentTimeIndex::toRowOrColumnIndex(fts.fluent, timeIndex, numberOfFluents, numberOfTimepoints);
                    // Always connect to the next timestep
                    size_t col = FluentTimeIndex::toRowOrColumnIndex(fts.fluent, timeIndex + 1, numberOfFluents, numberOfFluents);


                    LOG_INFO_S << "EdgeActivation for col: " << col << ", row: " << row << " requirement for: " << role.toString() << " roleRequirement: " << roleRequirement;
                    LOG_INFO_S << "Translates to: " << from->toString() << " to " << to->toString();
                    LOG_INFO_S << "Fluent: " << mLocations[fts.fluent]->toString();


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
    for(uint32_t roleIdx = 0; roleIdx < mActiveRoles.size(); ++roleIdx)
    {
        const Role& role = mRoles[ mActiveRoles[roleIdx] ];
        organization_model::facets::Robot robot(role.getModel(), mAsk);
        int32_t transportSupplyDemand = robot.getPayloadTransportSupplyDemand();
        if(transportSupplyDemand == 0)
        {
            throw std::invalid_argument("templ::solvers::csp::TransportNetwork: " +  role.getModel().toString() + " has"
                    " a transportSupplyDemand of 0 -- must be either positive of negative integer");
        }
        supplyDemand.push_back(transportSupplyDemand);
    }

    for(size_t t = 0; t < numberOfTimepoints; ++t)
    {
        for(size_t f = 0; f < numberOfFluents; ++f)
        {
            Gecode::SetVarArray multiEdge(*this, mActiveRoles.size());
            for(size_t i = 0; i < mActiveRoles.size(); ++i)
            {
                multiEdge[i] = mTimelines[i][t*numberOfFluents + f];
            }
            propagators::isValidTransportEdge(*this, multiEdge, supplyDemand, t, f, numberOfFluents);

            Gecode::SetAFC afc(*this, multiEdge, 0.99);
            afc.decay(*this, 0.95);

            // http://www.gecode.org/doc-latest/reference/group__TaskModelSetBranchVar.html
            branch(*this, multiEdge, Gecode::SET_VAR_AFC_MIN(afc), Gecode::SET_VAL_MIN_INC());
        }
    }

    for(size_t i = 0; i < mActiveRoles.size(); ++i)
    {
//        propagators::isPath(*this, mTimelines[i], numberOfTimepoints, mLocations.size());
    }
}

size_t TransportNetwork::getMaxResourceCount(const organization_model::ModelPool& pool) const
{
    size_t maxValue = std::numeric_limits<size_t>::min();
    organization_model::ModelPool::const_iterator cit = pool.begin();
    for(; cit != pool.end(); ++cit)
    {
        maxValue = std::max( maxValue, cit->second);
    }
    return maxValue;
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

    //// Check if resource requirements holds
    //for(size_t i = 0; i < mResourceRequirements.size(); ++i)
    //{
    //    ss << "assignment for:"
    //        << mResourceRequirements[i].toString()
    //        << " available resources: " << mMission.getAvailableResources().size()
    //        << std::endl;

    //    for(size_t mi = 0; mi < mMission.getAvailableResources().size(); ++mi)
    //    {
    //        Gecode::IntVar var = resourceDistribution(mi, i);
    //        var.assigned();
    //        Gecode::IntVarValues v( var );
    //        ss << "    "
    //            << mAvailableModels[mi].toString()
    //            << " : "
    //            << "assigned #" << v.val()
    //            << std::endl;
    //        ss << "Index: " << mi << "/" << i << " --> " << v.val() << std::endl;
    //    }
    //}
    // ss << "Array: " << resourceDistribution;

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
            const Gecode::SetVarArray& timeline = mTimelines[i];
            ss << timeline << std::endl;
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
    throw std::invalid_argument("templ::solvers::csp::TransportNetwork::getTimepointIdx: unknown timepoint given");
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

void TransportNetwork::addFunctionRequirement(const FluentTimeResource& fts, owlapi::model::IRI& function)
{
    // Find the function requirement index
    size_t index = 0;
    owlapi::model::IRIList::const_iterator cit = mResources.begin();
    for(; cit != mResources.end(); ++cit, ++index)
    {
        if(*cit == function)
        {
            break;
        }
    }

    // If function cannot be found add the function to the (known) required resources
    if(index >= mResources.size())
    {
        if( mAsk.ontology().isSubClassOf(function, organization_model::vocabulary::OM::Functionality()) )
        {
            LOG_INFO_S << "AUTO ADDED '" << function << "' to required resources";
            mResources.push_back(function);
        } else {
            throw std::invalid_argument("templ::solvers::csp::TransportNetwork: could not find the resource index for: '" + function.toString() + "' -- which is not a service class");
        }
    }
    LOG_DEBUG_S << "Using index: " << index;

    // identify the fluent time resource
    std::vector<FluentTimeResource>::iterator fit = std::find(mResourceRequirements.begin(), mResourceRequirements.end(), fts);
    if(fit == mResourceRequirements.end())
    {
        throw std::invalid_argument("templ::solvers::csp::TransportNetwork: could not find the fluent time resource: '" + fts.toString() + "'");
    }
    LOG_DEBUG_S << "Fluent before adding function requirement: " << fit->toString();

    // insert the function requirement
    fit->resources.insert(index);
    fit->maxCardinalities = organization_model::Algebra::max(fit->maxCardinalities, mAsk.getFunctionalSaturationBound(function) );
    LOG_DEBUG_S << "Fluent after adding function requirement: " << fit->toString();
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
