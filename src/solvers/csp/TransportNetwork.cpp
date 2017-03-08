#include "TransportNetwork.hpp"
#include <base-logging/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include <gecode/minimodel.hh>
#include <gecode/set.hh>

#include <gecode/gist.hh>
#include <organization_model/Algebra.hpp>
#include <organization_model/vocabularies/OM.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/symbols/object_variables/LocationCardinality.hpp>
#include <templ/SpaceTimeNetwork.hpp>
#include <iomanip>
#include <organization_model/facets/Robot.hpp>
#include <Eigen/Dense>

#include "ConstraintMatrix.hpp"
#include "propagators/IsPath.hpp"
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
        ss << hspace << "Timelines" << std::endl;
        ss << hspace  << TransportNetwork::toString(mTimelines);
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

TransportNetwork::Timelines TransportNetwork::getTimelines() const
{
    LOG_WARN_S << "GET TIMELINES";
    for(size_t i = 0; i < mTimelines.size(); ++i)
    {
        std::cout << mActiveRoleList[i].toString() << " #" << i << mTimelines[i] << std::endl;
        Gecode::SetVarArray::const_iterator cit = mTimelines[i].begin();
        size_t index = 0;
        for(; cit != mTimelines[i].end(); ++cit, ++index)
        {
            const Gecode::SetVar& var = *cit;
            if(!var.assigned())
            {
                throw std::invalid_argument("TransportNetwork::getTimelines: value is not assigned");
            } else {
                if(var.cardMax() == 1 && var.cardMin() == 1)
                {
                    Gecode::SetVarGlbValues currentVar(var);
                     std::cout << "    from: " << index << "    to: " << currentVar.val() << std::endl;
                     // index = timepointIdx*#locations + location
                     size_t fromLocationIdx = index % mLocations.size();
                     assert( mLocations.size() > fromLocationIdx );
                     symbols::constants::Location::Ptr fromLocation = mLocations[fromLocationIdx];

                     size_t  toLocationIdx = currentVar.val() % mLocations.size();

                     std::cout << "Locations: " << mLocations.size() << " to requested: " << toLocationIdx << std::endl;

                     assert( mLocations.size() > toLocationIdx );
                     symbols::constants::Location::Ptr toLocation = mLocations[toLocationIdx];

                     std::cout << "Readable" << std::endl;
                     std::cout<< "     from: " << fromLocation->toString() << "    to: " << toLocation->toString() << std::endl;
                }
            }
        }
    }
    //LOG_WARN_S << std::endl << toString(mTimelines) << std::endl;

    //size_t locationTimeSize = mLocations.size()*mTimepoints.size();

    TransportNetwork::Timelines timelines;
    //std::vector<int32_t> capacities;
    //for(size_t r = 0; r < locationTimeSize; ++r)
    //{
    //    FluentTimeIndex from = FluentTimeIndex::fromRowOrCol(r, mLocations.size(), mTimepoints.size());
    //    for(size_t c = 0; c < locationTimeSize; ++c)
    //    {
    //        FluentTimeIndex to = FluentTimeIndex::fromRowOrCol(r, mLocations.size(), mTimepoints.size());
    //        if(from.getFluentIndex() == to.getFluentIndex())
    //        {
    //            capacities.push_back( std::numeric_limits<int32_t>::max()/2);
    //        } else {
    //            capacities.push_back(0);
    //        }
    //    }
    //}

    //for(size_t roleIdx = 0; roleIdx < mActiveRoles.size(); ++roleIdx)
    //{
    //    const Role& role = mRoles[ mActiveRoles[roleIdx] ];
    //    LOG_WARN_S << "Timeline for: " << role.toString();

    //    TransportNetwork::Timeline finalRoleTimeline;

    //    Gecode::Matrix<Gecode::IntVarArray> timeline( mTimelines[roleIdx],
    //            locationTimeSize, locationTimeSize);

    //    int idx = 0;
    //    for(size_t rowIdx = 0; rowIdx < locationTimeSize; ++rowIdx)
    //    {
    //        FluentTimeIndex rowSpaceTimeIdx = FluentTimeIndex::fromRowOrCol(rowIdx, mLocations.size(), mTimepoints.size());

    //        for(size_t colIdx = 0; colIdx < locationTimeSize; ++colIdx)
    //        {
    //            Gecode::IntVar var = timeline(colIdx, rowIdx);
    //            if(!var.assigned())
    //            {
    //                throw std::runtime_error("templ::solvers::csp::TransportNetwork::getTimelines: "
    //                        "value has not been assigned for timeline of role:"
    //                        " '" + role.toString() + "'");
    //            }

    //            Gecode::IntVarValues v(var);
    //            if( v.val() == 1)
    //            {
    //                organization_model::facets::Robot robot(role.getModel(), mAsk);
    //                int32_t supplyDemand = robot.getPayloadTransportSupplyDemand();

    //                capacities[idx] = capacities[idx] + supplyDemand;
    //                LOG_WARN_S << "ROW: "<< rowIdx << " -- COL: " << colIdx;

    //                FluentTimeIndex colSpaceTimeIdx = FluentTimeIndex::fromRowOrCol(colIdx, mLocations.size(), mTimepoints.size());
    //                if(finalRoleTimeline.empty())
    //                {
    //                    // add from
    //                    SpaceTimePoint stp;
    //                    stp.first = mLocations[ rowSpaceTimeIdx.getFluentIndex() ];
    //                    stp.second = mTimepoints[ rowSpaceTimeIdx.getTimeIndex()];
    //                    finalRoleTimeline.push_back(stp);
    //                }

    //                // add to
    //                assert(colSpaceTimeIdx.first < mLocations.size());
    //                assert(colSpaceTimeIdx.second < mTimepoints.size());

    //                SpaceTimePoint stp;
    //                stp.first = mLocations[ colSpaceTimeIdx.getFluentIndex() ];
    //                stp.second = mTimepoints[ colSpaceTimeIdx.getTimeIndex()];
    //                finalRoleTimeline.push_back(stp);
    //            }

    //            ++idx;
    //        }
    //    }
    //    timelines[role] = finalRoleTimeline;
    //}

    //std::stringstream rowCol;
    //rowCol << std::endl;
    //bool isValid = true;
    //for(size_t r = 0; r < locationTimeSize; ++r)
    //{
    //    for(size_t c = 0; c < locationTimeSize; ++c)
    //    {
    //        size_t i = FluentTimeIndex::toArrayIndex(r,c, mLocations.size(), mTimepoints.size());

    //        rowCol << capacities[i] << "(" << std::setw(4) << i << ") ";

    //        if(capacities[i] < 0)
    //        {
    //            isValid = false;
    //            std::stringstream cc;
    //            cc << "capacity: " << capacities[i] << " at row: " << r << " , col: " << c;
    //            throw std::runtime_error("Invalid capacity of less than 0 -- " + cc.str());
    //        }
    //    }
    //    rowCol << std::endl;
    //}
    //if(isValid)
    //{
    //    throw std::runtime_error("Solution is valid");
    //}
    //LOG_WARN_S << rowCol.str();
    return timelines;
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
    , mCapacities(*this, (mLocations.size()+1)*(mLocations.size()+1)*mTimepoints.size()*mTimepoints.size(), 0, Gecode::Int::Limits::max)
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

    initializeRoleDistributionConstraints();
    enforceUnaryResourceUsage();

    // (C) Avoid computation of solutions that are redunant
    // Gecode documentation says however in 8.10.2 that "Symmetry breaking by
    // LDSB is not guaranteed to be complete. That is, a search may still return
    // two distinct solutions that are symmetric."
    //
    Gecode::Symmetries symmetries = identifySymmetries();

    Gecode::IntAFC afc(*this, mModelUsage, 0.99);
    afc.decay(*this, 0.95);

    Gecode::Gist::stopBranch(*this);
    branch(*this, mRoleUsage, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_MIN(), symmetries);
    //branch(*this, mRoleUsage, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_MIN(), symmetries);
    //branch(*this, mRoleUsage, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_MIN(), symmetries);
    //branch(*this, mModelUsage, Gecode::INT_VAR_AFC_MIN(afc), Gecode::INT_VAL_MIN());

    Gecode::Gist::stopBranch(*this);
    // see 8.14 Executing code between branchers
    branch(*this, &TransportNetwork::postRoleAssignments);

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
    mCapacities.update(*this, share, other.mCapacities);
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
            options.threads = 0;
            Gecode::Search::Cutoff * c = Gecode::Search::Cutoff::constant(70000);
            options.cutoff = c;

            //Gecode::BAB<TransportNetwork> searchEngine(distribution,options);
            //Gecode::DFS<TransportNetwork> searchEngine(this);
            Gecode::RBS< Gecode::BAB, TransportNetwork> searchEngine(distribution, options);

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

void TransportNetwork::postRoleAssignments(Gecode::Space& home)
{
    LOG_WARN_S << "POST ROLE ASSIGNMENTS";
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
    return activeRoles;
}

void TransportNetwork::postRoleAssignments()
{
    LOG_WARN_S << "POST ROLE ASSIGNMENTS: request status";
    (void) status();

    std::stringstream ss;
    ss << "Role usage:" << std::endl;
    for(size_t i = 0; i < mRoles.size(); ++i)
    {
        ss << std::setw(30) << std::left << mRoles[i].toString() << ": ";
        for(size_t r = 0; r < mResourceRequirements.size(); ++r)
        {
            ss << std::setw(10) << mRoleUsage[r*mRoles.size() + i] << " ";
        }
        ss << std::endl;
    }
    LOG_WARN_S << ss.str();

    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mResourceRequirements.size());

    //#############################################
    // construct timelines
    // ############################################
    // row = |timepoints|*location + timepoint-from
    // col = |timepoints|*location + timepoint-to
    //
    // i.e.
    //
    //        l0-t0  l0-t1 ... l0-tn l1-t0 l1-t1 ... l1-tn
    // l0-t0
    // l0-t1
    // ...
    // -- add an additional transfer location to allow for 'timegaps'
    mLocations.push_back(symbols::constants::Location::Ptr(new symbols::constants::Location("in-transfer")));

    size_t locationTimeSize = mLocations.size()*mTimepoints.size();
    mActiveRoles = getActiveRoles();

    LOG_INFO_S << "Adjacency list (locationTime) size: " << locationTimeSize << " -- for " << mRoles.size() << " roles; " << mActiveRoles.size() << " are active roles";

    // Collect all relevant location time values
    // which represents a superset of the final timeline
    std::vector<int> allowedLocationTimeValues;
    Role::List activeRoles;
    for(uint32_t roleIndex = 0; roleIndex < mRoles.size(); ++roleIndex)
    {
        // consider only active roles
        if(mActiveRoles.end() == std::find(mActiveRoles.begin(), mActiveRoles.end(), roleIndex))
        {
            LOG_DEBUG_S << "Is not an active role: " << mRoles[roleIndex].toString();
            continue;
        }
        const Role& role = mRoles[roleIndex];
        LOG_WARN_S << "Is an active role: " << role.toString();

        activeRoles.push_back(role);

        // A timeline describes the transitions in space time for a given role
        // A timeline is represented by an adjacency list, pointing from the current node
        // to the next -- given some temporal constraints
        // An empty list assignment means, there is no transition and at
        // maximum there can be one transition

        // Initialize timelines for all roles, i.e. here the current one
        Gecode::SetVarArray timeline(*this, locationTimeSize, Gecode::IntSet::empty, Gecode::IntSet(0,locationTimeSize-1),0,1);
        mTimelines.push_back(timeline);

        // DEBUG BEGIN
        using namespace solvers::temporal;
        std::vector<point_algebra::TimePoint::Ptr>::const_iterator cit = mTimepoints.begin();
        for(; cit != mTimepoints.end(); ++cit)
        {
            LOG_DEBUG_S << "Timepoints: " << (*cit)->toString();
        }
        for(size_t i = 0; i < mLocations.size(); ++i)
        {
            LOG_DEBUG_S << "Locations: " << mLocations[i]->toString();
        }
        // DEBUG END

        bool prevTimeIdxAvailable = false;
        uint32_t prevTimeIdx = 0;
        uint32_t prevLocationIdx = 0;

        for(int t = 0; t < mTimepoints.size(); ++t)
        {
            Gecode::SetVarArray timestep(*this, mLocations.size());
            std::stringstream ss;
            for(int l = 0; l < mLocations.size(); ++l)
            {
                int idx = t*mLocations.size() + l;
                Gecode::SetVar& edgeActivation = timeline[idx];
                // Use SetView to manipulate the edgeActivation in the
                // timeline
                Gecode::Set::SetView v(edgeActivation);
                // http://www.gecode.org/doc-latest/reference/classGecode_1_1Set_1_1SetView.html
                // set value to 'col'
                v.cardMin(*this, 0);
                v.cardMax(*this, 1);
                v.exclude(*this, 0, t*mLocations.size());
                v.exclude(*this, (t+2)*mLocations.size(), mTimepoints.size()*mLocations.size());
                ss << std::setw(25) << v;

                timestep[l] = edgeActivation;
            }

            Gecode::SetVar allUnion(*this);
            Gecode::rel(*this, Gecode::SOT_UNION, timestep, allUnion);
            Gecode::cardinality(*this, allUnion, 0,1);

            ss << std::endl;
            LOG_WARN_S << ss.str();
        }

        LOG_WARN_S << "TIMELINE DONE FOR " << role.toString();

        // Link the edge activation to the role requirement, i.e. make sure that
        // for each requirement the interval is 'activated'
        for(uint32_t requirementIndex = 0; requirementIndex < mResourceRequirements.size(); ++requirementIndex)
        {
            // Check if the particular role, is required to fulfil the
            // requirement
            Gecode::IntVar roleRequirement = roleDistribution(roleIndex, requirementIndex);
            if(!roleRequirement.assigned())
            {
                throw std::runtime_error("TransportNetwork: roleRequirement is not assigned");
            }
            Gecode::IntVarValues var(roleRequirement);
            // Check if role is required, i.e., does it map to the interval
            if(var.val() == 1)
            {
                const FluentTimeResource& fts = mResourceRequirements[requirementIndex];
                // index of the location is: fts.fluent
                point_algebra::TimePoint::Ptr from = fts.getInterval().getFrom();
                point_algebra::TimePoint::Ptr to = fts.getInterval().getTo();
                uint32_t fromIndex = getTimepointIndex( from );
                uint32_t toIndex = getTimepointIndex( to );

                // The timeline is now update for the full interval the
                // requirement is covering
                for(uint32_t timeIndex = fromIndex; timeIndex < toIndex; ++timeIndex)
                {
                    int allowed = timeIndex*mLocations.size() + fts.fluent;
                    std::cout << "ADDING: (fromTime: " << fromIndex << " toTime: " << toIndex <<") " << allowed << std::endl;
                    std::cout << "     location: " << fts.fluent << std::endl;

                    allowedLocationTimeValues.push_back(allowed);
                    // index of the location is fts.fluent
                    // edge index:
                    // row = timepointIdx*#ofLocations + from-location-offset
                    // col = (timepointIdx + 1) *#ofLocations + to-location-offset
                    //
                    // location (offset) = row % #ofLocations
                    // timepointIndex = (row - location(offset)) / #ofLocations
                    size_t row = FluentTimeIndex::toRowOrColumnIndex(fts.fluent, timeIndex, mLocations.size(), mTimepoints.size());
                    // Always connect to the next timestep
                    size_t col = FluentTimeIndex::toRowOrColumnIndex(fts.fluent, timeIndex + 1, mLocations.size(), mTimepoints.size());


                    LOG_INFO_S << "EdgeActivation for col: " << col << ", row: " << row << " requirement for: " << role.toString() << " roleRequirement: " << roleRequirement;
                    LOG_INFO_S << "Translates to: " << from->toString() << " to " << to->toString();
                    LOG_INFO_S << "Fluent: " << mLocations[fts.fluent]->toString();


                    // constraint between timeline and roleRequirement
                    // if 'roleRequirement' is given, then edgeActivation = col,
                    // else edgeActivation = -1
                    LOG_DEBUG_S << "Set SetVar in col: " << col << " and row " << row;
                    Gecode::SetVar& edgeActivation = timeline[row];
                    // Use SetView to manipulate the edgeActivation in the
                    // timeline
                    Gecode::Set::SetView v(edgeActivation);
                    // http://www.gecode.org/doc-latest/reference/classGecode_1_1Set_1_1SetView.html
                    // set value to 'col'
                    v.intersect(*this, col,col);
                    v.cardMin(*this, 1);
                    v.cardMax(*this, 1);
                    v.exclude(*this, 0, timeIndex*mLocations.size());
                    v.exclude(*this, (timeIndex+2)*mLocations.size(), mTimepoints.size()*mLocations.size());
                    //std::cout << "restricted to: " << edgeActivation << std::endl;
                    //std::cout << timeline << std::endl;
                }
                allowedLocationTimeValues.push_back(toIndex*mLocations.size() + fts.fluent);

                // Handle the transition between two requirements
                // Collect allowed waypoints: actually
                // For the transition interval allow also the source or the
                // target location (and any close? location)
                if(prevTimeIdxAvailable)
                {
                    std::cout << "PrevTime: " << prevTimeIdx << " toTime " << fromIndex << std::endl;
                    for(uint32_t timeIndex = prevTimeIdx + 1; timeIndex < fromIndex; ++timeIndex)
                    {
                        // Most general approach
                        for(uint32_t fluentIdx = 0; fluentIdx < mLocations.size(); ++fluentIdx)
                        {
                            int allowed = timeIndex*mLocations.size() + fluentIdx;
                            std::cout << "ADDING: waypoint" << allowed << std::endl;
                            allowedLocationTimeValues.push_back(allowed);
                        }

                        //int allowed = timeIndex*mLocations.size() + fts.fluent;
                        //std::cout << "ADDING: waypoint" << allowed << std::endl;
                        //allowedLocationTimeValues.push_back(allowed);

                        //allowed = timeIndex*mLocations.size() + prevLocationIdx;
                        //std::cout << "ADDING: waypoint" << allowed << std::endl;
                        //allowedLocationTimeValues.push_back(allowed);

                        //allowed = timeIndex*mLocations.size() + mLocations.size() - 1;
                        //std::cout << "ADDING: waypoint" << allowed << std::endl;
                        //allowedLocationTimeValues.push_back(allowed);
                    }
                }

                prevTimeIdx = toIndex;
                prevLocationIdx = fts.fluent;

                prevTimeIdxAvailable = true;
            }
        }
    }


    // Now allow role timelines can be restricted to the given superset
    Gecode::IntSet allowedIntSetValues(allowedLocationTimeValues.data(), allowedLocationTimeValues.size());
    for(size_t t = 0; t < mTimelines.size(); ++t)
    {
        Gecode::SetVarArray& timeline = mTimelines[t];
        //Gecode::SetVarArray::iterator it = timeline.begin();
        //for(; it != timeline.end(); ++it)
        //{
        //    Gecode::SetVar& var = *it;
        //    std::cout << var << " should be a subset of " << allowedIntSetValues << std::endl;
        //    // Variable should be a subset of the allowedIntSetValue
        //    rel(*this, var <= allowedIntSetValues || var == Gecode::IntSet::empty);
        //}

        // Make sure that the timeline for each role forms a path
        // This allows to account for feasible paths for immobile units as well
        // as mobile units to cover this area
        std::stringstream ss;
        {
            for(size_t t = 0; t < mTimepoints.size(); ++t)
            {
                for(size_t l = 0; l < mLocations.size(); ++l)
                {
                    ss << std::setw(25) << timeline[t*mLocations.size() + l] << " ";
                }
                ss << std::endl;
            }
            LOG_WARN_S << "POST TIMELINE: " << std::endl << ss.str() << std::endl;
        }
        //propagators::isPath(*this, timeline, mTimepoints.size(), mLocations.size());

        {
            std::stringstream ss;
            for(size_t t = 0; t < mTimepoints.size(); ++t)
            {
                for(size_t l = 0; l < mLocations.size(); ++l)
                {
                    ss << std::setw(25) << timeline[t*mLocations.size() + l] << " ";
                }
                ss << std::endl;
            }
            LOG_WARN_S << "POST TIMELINE: " << std::endl << ss.str() << std::endl;
        }
    }

    LOG_WARN_S << "COMPLETED TIMELINE POSTING";

    mActiveRoleList = activeRoles;

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
    for(size_t i = 0; i < mActiveRoles.size(); ++i)
    {
        Gecode::SetAFC afc(*this, mTimelines[i], 0.99);
        afc.decay(*this, 0.95);

        Gecode::Gist::stopBranch(*this);

        // http://www.gecode.org/doc-latest/reference/group__TaskModelSetBranchVar.html
        //branch(*this, mTimelines[i], Gecode::SET_VAR_AFC_MIN(afc), Gecode::SET_VAL_MIN_INC());
        branch(*this, mTimelines[i], Gecode::SET_VAR_MAX_MAX(), Gecode::SET_VAL_MAX_EXC());
        Gecode::Gist::stopBranch(*this);
        //branch(*this, mTimelines[i], Gecode::SET_VAR_MIN_MIN(), Gecode::SET_VAL_MAX_INC());
        //branch(*this, mTimelines[i], Gecode::SET_VAR_NONE(), Gecode::SET_VAL_MED_INC());
    }

    //LOG_WARN_S << mTimelines[0];

    //LOG_WARN_S << "POST FLOW CAPACITIES";
    //branch(*this, &TransportNetwork::postFlowCapacities);
}

void TransportNetwork::postRoleTimelines(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).postRoleTimelines();
}

void TransportNetwork::postRoleTimelines()
{
//    LOG_WARN_S << "POST ROLE TIMELINES";
//    if(mActiveRoles.empty())
//    {
//        LOG_WARN_S << "NO ACTIVE ROLES";
//    }
//    (void) status();
//
//    // Check for each timeline -- which is associated with a role
//    for(size_t i = 0;  i < mTimelines.size(); ++i)
//    {
//        size_t locationTimeSize = mLocations.size()*mTimepoints.size();
//        LOG_DEBUG_S << "locations" << mLocations.size() << " timepoints " << mTimepoints.size();
//        AdjacencyList timeline = mTimelines[i];
//
//        // compute the adjacency list from the existing matrix
//        // identify the starting point and end point
//        std::vector< std::pair<size_t,size_t> > path;
//        Gecode::IntVarArgs adjacencyList;
//        for(size_t row = 0; row < locationTimeSize; ++row)
//        {
//            std::vector<int> neighbours;
//            for(size_t col = 0; col < locationTimeSize; ++col)
//            {
//                Gecode::IntVar var = timeline(col, row);
//                if(var.assigned())
//                {
//                    Gecode::IntVarValues v(var);
//                    if(v.val() == 1)
//                    {
//                        path.push_back( std::pair<size_t,size_t>(col,row) );
//                        break; // break the loop since there can be only one connection, so goto the next row
//                    }
//                }
//            }
//        }
//
//        if(!path.empty())
//        {
//            size_t pathStartRow = path.front().second;
//            size_t pathEndRow = path.back().second;
//            size_t numberOfRows = locationTimeSize;
//
//            // No path segments before the first requirement
//            {
//                Gecode::IntVarArgs args;
//                for(size_t rowIndex = 0; rowIndex < pathStartRow; ++rowIndex)
//                {
//                   args << timeline.row(rowIndex);
//                }
//                LOG_DEBUG_S << "graph constraint: no edges to/before first node";
//                rel(*this, sum(args) == 0);
//            }
//
//            // No path segments are needed after the last requirement
//            {
//                Gecode::IntVarArgs args;
//                for(size_t rowIndex = pathEndRow+1; rowIndex < numberOfRows; ++rowIndex)
//                {
//                    args << timeline.row(rowIndex);
//                }
//                LOG_DEBUG_S << "graph constraint: no edges from/after last node";
//                rel(*this, sum(args) == 0);
//            }
//
//            // For each element in the path except for end and start there needs to
//            // be a connection in the range of the two given edges
//            for(size_t rowIndex = pathStartRow; rowIndex <= pathEndRow; ++rowIndex)
//            {
//                // assume an already specified rel(*this, sum( timeline.row(rowIndex)) <= 1);
//                // row index matches the required target spatio-temporal tuple,
//                // which needs to be originated by another (therefore target of
//                // another must contain this tuple --> col(rowIndex) )
//                if(rowIndex > pathStartRow )
//                {
//                    // edges need to form a path, thus
//                    // from each vertex except for the end vertex the
//                    LOG_DEBUG_S << "graph constraint: edges from a path: sum of row: " << rowIndex << " == sum of col " << rowIndex;
//                    rel(*this, sum( timeline.row(rowIndex)) == sum (timeline.col(rowIndex)) );
//                }
//
//                // for each timestep there needs to be a path segment
//                if(rowIndex < pathEndRow && rowIndex%mLocations.size() == 0)
//                {
//                    // fc,tc, fr, tr: from column to column, from row to row
//                    // [tc,tc) -- open interval thus tc is not included!
//                    Gecode::IntVarArgs args;
//                    size_t fc = rowIndex + mLocations.size();
//                    size_t tc = fc + mLocations.size();
//                    size_t fr = rowIndex;
//                    size_t tr = rowIndex + mLocations.size();
//                    args << timeline.slice(fc, tc, fr, tr);
//                    LOG_DEBUG_S << "graph constraint: one edge between all involved timepoints";
//                    rel(*this, sum(args) == 1);
//                }
//            }
//
//            for(size_t nodeIndex = 0; nodeIndex < path.size(); ++nodeIndex)
//            {
//                size_t col = path[nodeIndex].first;
//                size_t row = path[nodeIndex].second;
//                // last affected row that is temporally concurrent with the
//                // current row
//
//                // If not start node
//                if(nodeIndex != 0)
//                {
//                    // Require the node 'row' to be connected to 'col'
//                    // i.e. previous edge needs to be directed to this one or
//                    // otherwise interpreted
//                    LOG_DEBUG_S << "graph constraint: incoming edge requirement: " << row  << " sum to 1";
//                    rel(*this, sum( timeline.col(row) ) == 1);
//                }
//
//                // If not end node then this node has to have an outgoing edge
//                if(nodeIndex != path.size() -1)
//                {
//                    LOG_WARN_S << "graph constraint: outgoing edge requirement: require row: " << col  << " sum to 1";
//                    rel(*this, sum( timeline.row(col) ) == 1);
//                }
//            }
//        }
//    }
}

void TransportNetwork::postFlowCapacities(Gecode::Space& home)
{
    static_cast<TransportNetwork&>(home).postFlowCapacities();
}

void TransportNetwork::postFlowCapacities()
{
    LOG_WARN_S << "GET STATUS FLOW FLOW CAPACITIES";
    (void) status();
    LOG_WARN_S << "POST FLOW CAPACITIES";
    for(size_t i = 0; i < mTimelines.size(); ++i)
    {
        LOG_WARN_S << "Timeline: " << mTimelines[i];
    }

    // Compute the multi commodity flow
    LOG_WARN_S << "POST MULTI COMMODITY FLOW: timeline size: " << mTimelines.size() << ", active roles: " << mActiveRoleList.size();
    assert(mTimelines.size() == mActiveRoleList.size());
    propagators::multiCommodityFlow(*this, mActiveRoleList, mTimelines, mTimepoints.size(), mLocations.size(), mpMission->getOrganizationModelAsk());

//
//
//    // Consumer vs. Providers
//    size_t locationTimeSize = mLocations.size()*mTimepoints.size();
//    size_t  numberOfRows = locationTimeSize;
//
//    typedef Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;
//    MatrixXi capacities = MatrixXi::Zero(locationTimeSize, locationTimeSize);
//
//    Gecode::Matrix< Gecode::IntVarArray > capacityMatrix(mCapacities, locationTimeSize, locationTimeSize);
//
//    for(size_t rowIndex = 0; rowIndex < locationTimeSize; ++rowIndex)
//    {
//        FluentTimeIndex rowSpaceTimeIdx = FluentTimeIndex::fromRowOrCol(rowIndex, mLocations.size(), mTimepoints.size());
//
//        LOG_WARN_S << "row: " << rowIndex <<  " " << mLocations[rowSpaceTimeIdx.getFluentIndex()]->toString();
//        LOG_WARN_S << "                          " << mTimepoints[rowSpaceTimeIdx.getTimeIndex()]->toString();
//
//        for(size_t colIndex = 0; colIndex < locationTimeSize; ++colIndex)
//        {
//            bool isSameLocation = false;
//
//            FluentTimeIndex colSpaceTimeIdx = FluentTimeIndex::fromRowOrCol(colIndex, mLocations.size(), mTimepoints.size());
//
//            LOG_WARN_S << "col: " << colIndex <<  " " << mLocations[colSpaceTimeIdx.getFluentIndex()]->toString();
//            LOG_WARN_S << "       " << mTimepoints[colSpaceTimeIdx.getTimeIndex()]->toString();
//
//            if( rowSpaceTimeIdx.getFluentIndex() == colSpaceTimeIdx.getFluentIndex())
//            {
//                LOG_WARN_S << "Is same location";
//                isSameLocation = true;
//                if(rowSpaceTimeIdx.getTimeIndex() + 1 == colSpaceTimeIdx.getTimeIndex())
//                {
//                    LOG_WARN_S << "Updating: " << rowIndex << "/" << colIndex;
//                    capacities(rowIndex, colIndex) = std::numeric_limits<int32_t>::max();
//                } else {
//                    LOG_WARN_S << "Not subsequent timesteps";
//                }
//            }
//
//            std::vector<uint32_t>::const_iterator cit = mActiveRoles.begin();
//            size_t timelineIndex = 0;
//            for(; cit != mActiveRoles.end(); ++cit, ++timelineIndex)
//            {
//                const Role& role = mRoles[*cit];
//
//                if(!isSameLocation)
//                {
//                        Gecode::Matrix<Gecode::IntVarArray> timeline(mTimelines[timelineIndex], numberOfRows, numberOfRows);
//                        // check assignment
//                        Gecode::IntVar pathElement = timeline(colIndex, rowIndex);
//                        if(pathElement.assigned())
//                        {
//                            Gecode::IntVarValues v(pathElement);
//                            if(v.val() == 1)
//                            {
//
//                                organization_model::facets::Robot robot(role.getModel(), mpMission->getOrganizationModelAsk());
//                                if(robot.isMobile())
//                                {
//                                    // Provider
//                                    uint32_t capacity = robot.getPayloadTransportCapacity();
//                                    capacities(rowIndex, colIndex) += capacity;
//                                    LOG_WARN_S << "INCREASE CAPACITY OF: " << capacity << " at " << rowIndex << " " << colIndex << " cap: " << capacities(rowIndex, colIndex);
//                                } else {
//                                    // Consumer
//                                    capacities(rowIndex, colIndex) -= 1; //capacity;
//                                    LOG_WARN_S << "REDUCE CAPACITY OF: " << " at " << rowIndex << " " << colIndex << " cap" << capacities(rowIndex, colIndex);
//                                }
//                            }
//                        }
//                    }
//            }
//            int capacity = capacities(rowIndex, colIndex);
//            Gecode::IntVar capacityVar = capacityMatrix(colIndex, rowIndex);
//            if(capacity == std::numeric_limits<int32_t>::max())
//            {
//                rel(*this, capacityVar, Gecode::IRT_EQ, Gecode::Int::Limits::max);
//            } else {
//                rel(*this, capacityVar, Gecode::IRT_EQ, capacity);
//            }
//
//            rel(*this, capacityVar, Gecode::IRT_GQ, 0);
//        }
//    }
//
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
        for(int i = 0; i < mTimelines.size(); ++i)
        {
            const Gecode::SetVarArray& timeline = mTimelines[i];
            ss << timeline << std::endl;
        }

        Timelines timelines = convertToTimelines(mActiveRoleList, mTimelines);
        ss << "Number of timelines: " << timelines.size() << " active roles -- " << mActiveRoleList.size() << std::endl;
        ss << "Current timelines:" << std::endl << toString(timelines) << std::endl;
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

std::string TransportNetwork::toString(const Timeline& timeline, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << "TIMELINE --" << std::endl;
    Timeline::const_iterator cit = timeline.begin();
    for(; cit != timeline.end(); ++cit)
    {
        const SpaceTimePoint& stp = *cit;
        assert(stp.first);
        if(stp.first)
        {
            ss << hspace <<  stp.first->toString();
        } else {
            ss << " -- " << "unknown location";
        }

        if(stp.second)
        {
            ss << " -- " << stp.second->toString();
        } else {
            ss << " -- " << "unknown timepoint";
        }

        ss << std::endl;
    }
    return ss.str();
}

std::string TransportNetwork::toString(const Timelines& timelines, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << "TIMELINES --" << std::endl;

    Timelines::const_iterator cit = timelines.begin();
    for(; cit != timelines.end(); ++cit)
    {
        const Role& role = cit->first;
        const Timeline& timeline = cit->second;

        ss << hspace << role.toString() << std::endl;
        ss << toString(timeline, indent + 4);
    }
    return ss.str();
}

TransportNetwork::Timeline TransportNetwork::convertToTimeline(const AdjacencyList& list) const
{
    Timeline timeline;
    int expectedTargetIdx;
    for(int idx = 0; idx < list.size(); ++idx)
    {
        const Gecode::SetVar& var = list[idx];
        if(!var.assigned())
        {
            SpaceTimePoint stp();
        //    throw std::invalid_argument("templ::solvers::csp::TransportNetwork::toString: cannot compute timeline, value is not assigned");
        }

        if(var.cardMax() == 1 && var.cardMin() == 1)
        {
            if(!timeline.empty())
            {
                if(expectedTargetIdx != idx)
                {
                    std::stringstream ss;
                    ss << "templ::solvers::csp::TransportNetwork::toString: timeline is invalid: ";
                    ss << " expected idx " << expectedTargetIdx << ", but got " << idx;
                    LOG_WARN_S << ss.str();
                    //throw std::runtime_error(ss.str());
                }
            }
            Gecode::SetVarGlbValues value(var);
            uint32_t targetIdx = value.val();
            expectedTargetIdx = targetIdx;

            uint32_t fromLocationIdx = idx % mLocations.size();
            uint32_t fromTimeIdx = (idx - fromLocationIdx)/mLocations.size();

            uint32_t toLocationIdx = targetIdx % mLocations.size();
            uint32_t toTimeIdx = (targetIdx - fromLocationIdx)/mLocations.size();

            //if(timeline.empty())
            //{
                SpaceTimePoint sourceStp(mLocations[fromLocationIdx], mTimepoints[fromTimeIdx]);
                timeline.push_back(sourceStp);
            //}

            SpaceTimePoint targetStp(mLocations[toLocationIdx], mTimepoints[toTimeIdx]);
            timeline.push_back(targetStp);
        } else {
            // empty set
            continue;
        }
    }
    LOG_WARN_S << "TIMELINE OF SIZE: " << timeline.size();
    return timeline;
}

TransportNetwork::Timelines TransportNetwork::convertToTimelines(const Role::List& roles, const ListOfAdjacencyLists& lists) const
{
    assert(!roles.empty());

    Timelines timelines;
    if(roles.size() != lists.size())
    {
        throw std::invalid_argument("templ::solvers::csp::TransportNetwork::convertToTimelines: size of roles does not equal size of adjacency lists");
    }

    for(size_t i = 0; i < roles.size(); ++i)
    {
        timelines[ roles[i] ] = convertToTimeline(lists[i]);
    }
    LOG_WARN_S << "TIMELINES OF SIZE: " << timelines.size();
    return timelines;
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
    size_t index = 0;
    owlapi::model::IRIList::const_iterator cit = mResources.begin();
    for(; cit != mResources.end(); ++cit, ++index)
    {
        if(*cit == function)
        {
            break;
        }
    }
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

    std::vector<FluentTimeResource>::iterator fit = std::find(mResourceRequirements.begin(), mResourceRequirements.end(), fts);
    if(fit == mResourceRequirements.end())
    {
        throw std::invalid_argument("templ::solvers::csp::TransportNetwork: could not find the fluent time resource: '" + fts.toString() + "'");
    }
    LOG_DEBUG_S << "Fluent before adding function requirement: " << fit->toString();
    // insert the resource requirement
    fit->resources.insert(index);
    fit->maxCardinalities = organization_model::Algebra::max(fit->maxCardinalities, mAsk.getFunctionalSaturationBound(function) );
    LOG_DEBUG_S << "Fluent after adding function requirement: " << fit->toString();
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
