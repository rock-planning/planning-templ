#include "ModelDistribution.hpp"
#include <base/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include <gecode/minimodel.hh>

#include <gecode/gist.hh>
#include <organization_model/Algebra.hpp>
#include <organization_model/vocabularies/OM.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/symbols/object_variables/LocationCardinality.hpp>
#include <templ/solvers/csp/ConstraintMatrix.hpp>

namespace templ {
namespace solvers {
namespace csp {

ModelDistribution::SearchState::SearchState(const Mission::Ptr& mission)
    : mpMission(mission)
    , mpInitialState(NULL)
    , mpSearchEngine(NULL)
    , mType(OPEN)
{
    assert(mpMission);
    mpInitialState = ModelDistribution::Ptr(new ModelDistribution(mpMission));
    mpSearchEngine = ModelDistribution::BABSearchEnginePtr(new Gecode::BAB<ModelDistribution>(mpInitialState.get()));
}

ModelDistribution::SearchState::SearchState(const ModelDistribution::Ptr& modelDistribution,
        const ModelDistribution::BABSearchEnginePtr& searchEngine)
    : mpMission(modelDistribution->mpMission)
    , mpInitialState(modelDistribution)
    , mpSearchEngine(searchEngine)
    , mType(OPEN)
{
    assert(mpMission);
    assert(mpInitialState);
    if(!mpSearchEngine)
    {
        mpSearchEngine = ModelDistribution::BABSearchEnginePtr(new Gecode::BAB<ModelDistribution>(mpInitialState.get()));
    }
}

ModelDistribution::SearchState ModelDistribution::SearchState::next() const
{
    if(!getInitialState())
    {
        throw std::runtime_error("templ::solvers::csp::ModelDistribution::SearchState::next: "
                " next() called on an unitialized search state");
    }

    SearchState searchState(mpInitialState, mpSearchEngine);
    ModelDistribution* solvedDistribution = mpSearchEngine->next();
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

ModelDistribution::Solution ModelDistribution::getSolution() const
{
    Solution solution;
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
                throw std::runtime_error("templ::solvers::csp::ModelDistribution::getSolution: value has not been assigned");
            }

            Gecode::IntVarValues v( var );

            modelPool[ mAvailableModels[mi] ] = v.val();
        }

        solution[ mResourceRequirements[i] ] = modelPool;
    }

    return solution;
}

std::set< std::vector<uint32_t> > ModelDistribution::toCSP(const organization_model::ModelPoolSet& combinations) const
{
    std::set< std::vector<uint32_t> > csp_combinations;
    organization_model::ModelPoolSet::const_iterator cit = combinations.begin();
    for(; cit != combinations.end(); ++cit)
    {
        csp_combinations.insert( toCSP(*cit) );
    }
    return csp_combinations;
}

std::vector<uint32_t> ModelDistribution::toCSP(const organization_model::ModelPool& combination) const
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

uint32_t ModelDistribution::systemModelToCSP(const owlapi::model::IRI& model) const
{
    owlapi::model::IRIList::const_iterator cit = std::find(mAvailableModels.begin(),
            mAvailableModels.end(), model);

    if(cit == mAvailableModels.end())
    {
        throw std::invalid_argument("templ::solvers::csp::ModelDistribution::systemModelToCSP:"
                " unknown model '" + model.toString() );
    } else {
        return (cit - mAvailableModels.begin());
    }

}

ModelDistribution::ModelDistribution(const templ::Mission::Ptr& mission)
    : Gecode::Space()
    , mpMission(mission)
    , mModelPool(mpMission->getAvailableResources())
    , mAsk(mpMission->getOrganizationModel(), mpMission->getAvailableResources(), true)
    , mResources(mpMission->getRequestedResources())
    , mIntervals(mpMission->getTimeIntervals())
    , mLocations(mpMission->getLocations())
    , mResourceRequirements(getResourceRequirements())
    , mModelUsage(*this, /*# of models*/ mpMission->getAvailableResources().size()*
            /*# of fluent time services*/mResourceRequirements.size(), 0, getMaxResourceCount(mModelPool))
    , mAvailableModels(mpMission->getModels())
{
    assert( mpMission->getOrganizationModel() );
    assert(!mIntervals.empty());

    ConstraintMatrix constraintMatrix(mAvailableModels);

    if(mResourceRequirements.empty())
    {
        throw std::invalid_argument("templ::solvers::csp::ModelDistribution: no resource requirements given");
    }

    LOG_INFO_S << "ModelDistribution CSP Problem Construction" << std::endl
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
    //
    // (B) General resource constraints
    //     - identify overlapping fts, limit resources for these (TODO: better
    //     indentification of overlapping requirements)
    //
    //
    // (C) Minimal resource constraints associated with Time-Location
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
                        /// Consider additional resource cardinality constraint
                        LOG_DEBUG_S << "Check extra min cardinality for " << mAvailableModels[mi];
                        organization_model::ModelPool::const_iterator cardinalityIt = fts.minCardinalities.find( mAvailableModels[mi] );
                        if(cardinalityIt != fts.minCardinalities.end())
                        {
                            minCardinality = cardinalityIt->second;
                            LOG_DEBUG_S << "Found extra resource cardinality constraint: " << std::endl
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

                // Extensional constraints, i.e. specifying the allowed
                // combinations
                organization_model::ModelPoolSet allowedCombinations = fts.getDomain();
                appendToTupleSet(mExtensionalConstraints[requirementIndex], allowedCombinations);

                // there can be no empty assignment for a service
                rel(*this, sum( resourceDistribution.row(requirementIndex) ) > 0);

                // This can be equivalently modelled using a linear constraint
                // Gecode::IntArgs c(mAvailableModels.size());
                // for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
                //    c[mi] = 1;
                // linear(*this, c, resourceDistribution.row(requirementIndex), Gecode::IRT_GR, 0);
            }
        }

        for(auto pair : mExtensionalConstraints)
        {
            uint32_t requirementIndex = pair.first;
            Gecode::TupleSet& tupleSet = pair.second;

            tupleSet.finalize();
            extensional(*this, resourceDistribution.row(requirementIndex), tupleSet);
        }

        LOG_INFO_S << constraintMatrix.toString();
    }
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

    branch(*this, mModelUsage, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_SPLIT_MIN());
    branch(*this, mModelUsage, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_SPLIT_MIN());
    branch(*this, mModelUsage, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_SPLIT_MIN());

    //Gecode::Gist::Print<ModelDistribution> p("Print solution");
    //Gecode::Gist::Options o;
    //o.inspect.click(&p);
    //Gecode::Gist::bab(this, o);

}

ModelDistribution::ModelDistribution(bool share, ModelDistribution& other)
    : Gecode::Space(share, other)
    , mpMission(other.mpMission)
    , mModelPool(other.mModelPool)
    , mAsk(other.mAsk)
    , mServices(other.mServices)
    , mIntervals(other.mIntervals)
    , mLocations(other.mLocations)
    , mResourceRequirements(other.mResourceRequirements)
    , mAvailableModels(other.mAvailableModels)
{
    assert( mpMission->getOrganizationModel() );
    assert(!mIntervals.empty());
    mModelUsage.update(*this, share, other.mModelUsage);
}

Gecode::Space* ModelDistribution::copy(bool share)
{
    return new ModelDistribution(share, *this);
}

std::vector<ModelDistribution::Solution> ModelDistribution::solve(const templ::Mission::Ptr& mission)
{
    SolutionList solutions;

    mission->validateForPlanning();

    assert(mission->getOrganizationModel());
    assert(!mission->getTimeIntervals().empty());

    ModelDistribution* distribution = new ModelDistribution(mission);
    {
        Gecode::BAB<ModelDistribution> searchEngine(distribution);
        //Gecode::DFS<ModelDistribution> searchEngine(this);

        ModelDistribution* best = NULL;
        while(ModelDistribution* current = searchEngine.next())
        {
            delete best;
            best = current;

            using namespace organization_model;

            LOG_INFO_S << "Solution found:" << current->toString();
            solutions.push_back(current->getSolution());
        }

        if(best == NULL)
        {
            delete distribution;
            throw std::runtime_error("templ::solvers::csp::ModelDistribution::solve: no solution found");
        }
    }

    delete best;
    delete distribution;

    return solutions;
}

void ModelDistribution::appendToTupleSet(Gecode::TupleSet& tupleSet, const organization_model::ModelPoolSet& combinations) const
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

size_t ModelDistribution::getFluentIndex(const FluentTimeResource& fluent) const
{
    std::vector<FluentTimeResource>::const_iterator ftsIt = std::find(mResourceRequirements.begin(), mResourceRequirements.end(), fluent);
    if(ftsIt != mResourceRequirements.end())
    {
        int index = ftsIt - mResourceRequirements.begin();
        assert(index >= 0);
        return (size_t) index;
    }

    throw std::runtime_error("templ::solvers::csp::ModelDistribution::getFluentIndex: could not find fluent index for '" + fluent.toString() + "'");
}

size_t ModelDistribution::getResourceModelIndex(const owlapi::model::IRI& model) const
{
    owlapi::model::IRIList::const_iterator cit = std::find(mAvailableModels.begin(), mAvailableModels.end(), model);
    if(cit != mAvailableModels.end())
    {
        int index = cit - mAvailableModels.begin();
        assert(index >= 0);
        return (size_t) index;
    }

    throw std::runtime_error("templ::solvers::csp::ModelDistribution::getResourceModelIndex: could not find model index for '" + model.toString() + "'");
}

const owlapi::model::IRI& ModelDistribution::getResourceModelFromIndex(size_t index) const
{
    if(index < mAvailableModels.size())
    {
        return mAvailableModels.at(index);
    }
    throw std::invalid_argument("templ::solvers::csp::ModelDistribution::getResourceModelIndex: index is out of bounds");
}

size_t ModelDistribution::getResourceModelMaxCardinality(size_t index) const
{
    organization_model::ModelPool::const_iterator cit = mModelPool.find(getResourceModelFromIndex(index));
    if(cit != mModelPool.end())
    {
        return cit->second;
    }
    throw std::invalid_argument("templ::solvers::csp::ModelDistribution::getResourceModelMaxCardinality: model not found");
}

std::vector<FluentTimeResource> ModelDistribution::getResourceRequirements() const
{
    if(mIntervals.empty())
    {
        throw std::runtime_error("solvers::csp::ModelDistribution::getResourceRequirements: no time intervals available"
                " -- make sure you called prepareTimeIntervals() on the mission instance");
    }


    std::vector<FluentTimeResource> requirements;

    using namespace templ::solvers::temporal;
    point_algebra::TimePointComparator timepointComparator(mpMission->getTemporalConstraintNetwork());

    // Iterate over all existing persistence conditions
    // -- pick the ones relating to location-cardinality function
    const std::vector<PersistenceCondition::Ptr>& conditions = mpMission->getPersistenceConditions();
    std::vector<PersistenceCondition::Ptr>::const_iterator cit = conditions.begin();
    for(; cit != conditions.end(); ++cit)
    {
        PersistenceCondition::Ptr p = *cit;


        symbols::StateVariable stateVariable = p->getStateVariable();
        if(stateVariable.getFunction() != symbols::ObjectVariable::TypeTxt[symbols::ObjectVariable::LOCATION_CARDINALITY] )
        {
            continue;
        }

        owlapi::model::IRI resourceModel(stateVariable.getResource());
        symbols::ObjectVariable::Ptr objectVariable = dynamic_pointer_cast<symbols::ObjectVariable>(p->getValue());
        symbols::object_variables::LocationCardinality::Ptr locationCardinality = dynamic_pointer_cast<symbols::object_variables::LocationCardinality>(objectVariable);

        Interval interval(p->getFromTimePoint(), p->getToTimePoint(),timepointComparator);
        {
            std::vector<Interval>::const_iterator iit = std::find(mIntervals.begin(), mIntervals.end(), interval);
            if(iit == mIntervals.end())
            {
                LOG_INFO_S << "Size of intervals: " << mIntervals.size();
                throw std::runtime_error("templ::solvers::csp::ModelDistribution::getResourceRequirements: could not find interval: '" + interval.toString() + "'");
            }

            owlapi::model::IRIList::const_iterator sit = std::find(mResources.begin(), mResources.end(), resourceModel);
            if(sit == mResources.end())
            {
                throw std::runtime_error("templ::solvers::csp::ModelDistribution::getResourceRequirements: could not find service: '" + resourceModel.toString() + "'");
            }

            symbols::constants::Location::Ptr location = locationCardinality->getLocation();
            std::vector<symbols::constants::Location::Ptr>::const_iterator lit = std::find(mLocations.begin(), mLocations.end(), location);
            if(lit == mLocations.end())
            {
                throw std::runtime_error("templ::solvers::csp::ModelDistribution::getResourceRequirements: could not find location: '" + location->toString() + "'");
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
                ftr.minCardinalities[ resourceModel ] = locationCardinality->getCardinality();
            } else {
                LOG_WARN_S << "Unsupported state variable: " << resourceModel;
                continue;
            }

            ftr.maxCardinalities = organization_model::Algebra::max(ftr.maxCardinalities, ftr.minCardinalities);
            requirements.push_back(ftr);
            LOG_DEBUG_S << ftr.toString();
        }
    }

    compact(requirements);
    return requirements;
}

void ModelDistribution::compact(std::vector<FluentTimeResource>& requirements) const
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

size_t ModelDistribution::getMaxResourceCount(const organization_model::ModelPool& pool) const
{
    size_t maxValue = std::numeric_limits<size_t>::min();
    organization_model::ModelPool::const_iterator cit = pool.begin();
    for(; cit != pool.end(); ++cit)
    {
        maxValue = std::max( maxValue, cit->second);
    }
    return maxValue;
}

std::string ModelDistribution::toString() const
{
    std::stringstream ss;
    ss << "ModelDistribution: #" << std::endl;
    //Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, mModelPool.size(), mResourceRequirements.size());

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
    ss << "Current model usage: " << mModelUsage;

    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const ModelDistribution::Solution& solution)
{
    ModelDistribution::Solution::const_iterator cit = solution.begin();
    size_t count = 0;
    os << "Solution" << std::endl;
    for(; cit != solution.end(); ++cit)
    {
        const FluentTimeResource& fts = cit->first;
        os << "--- requirement #" << count++ << std::endl;
        os << fts.toString() << std::endl;

        const organization_model::ModelPool& modelPool = cit->second;
        os << modelPool.toString() << std::endl;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const ModelDistribution::SolutionList& solutions)
{
    ModelDistribution::SolutionList::const_iterator cit = solutions.begin();
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

void ModelDistribution::addFunctionRequirement(const FluentTimeResource& fts, owlapi::model::IRI& function)
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
            throw std::invalid_argument("templ::solvers::csp::ModelDistribution: could not find the resource index for: '" + function.toString() + "' -- which is not a service class");
        }
    }
    LOG_DEBUG_S << "Using index: " << index;

    std::vector<FluentTimeResource>::iterator fit = std::find(mResourceRequirements.begin(), mResourceRequirements.end(), fts);
    if(fit == mResourceRequirements.end())
    {
        throw std::invalid_argument("templ::solvers::csp::ModelDistribution: could not find the fluent time resource: '" + fts.toString() + "'");
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
