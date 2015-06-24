#include "ModelDistribution.hpp"
#include <base/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include <gecode/minimodel.hh>
#include <organization_model/Algebra.hpp>

namespace templ {
namespace solvers {
namespace csp {

ModelDistribution* ModelDistribution::solve()
{
    Gecode::BAB<ModelDistribution> searchEngine(this);
    //Gecode::DFS<ResourceMatch> searchEngine(this);

    ModelDistribution* best = NULL;
    while(ModelDistribution* current = searchEngine.next())
    {
        delete best;
        best = current;

        using namespace organization_model;
        ModelPoolDelta allConsumedResources;
        // Check if resource requirements holds
        for(int i = 0; i < mSetAssignments.size(); ++i)
        {
            for(Gecode::SetVarGlbValues v(mSetAssignments[i]); v(); ++v)
            {
                // v.val() --> the assigned value
                ModelCombination mc = mDomain[v.val()];
                ModelPool consumedResources = OrganizationModel::combination2ModelPool(mc);

                allConsumedResources = Algebra::sum(allConsumedResources, consumedResources);
            }
        }
        ModelPoolDelta diff = Algebra::delta(mMission.getResources(), allConsumedResources);
        if(diff.isNegative())
        {
            LOG_DEBUG_S << "Invalid solution found: exceeding available resources";
        } else {
            break;
        }
    }

    if(best == NULL)
    {
        throw std::runtime_error("templ::solvers::csp::ModelDistribution::solve: no solution found");
    }

    return best;
}

Solution ModelDistribution::getSolution() const
{
    return Solution();
}

organization_model::ModelCombinationSet ModelDistribution::getDomain(const FluentTimeService& requirement) const
{
    std::set<organization_model::Service> services;
    services.insert( organization_model::Service( mServices[requirement.service] ) );

    return mAsk.getResourceSupport(services);
}


std::set< std::vector<uint32_t> > ModelDistribution::toCSP(const organization_model::ModelCombinationSet& combinations) const
{
    std::set< std::vector<uint32_t> > csp_combinations;
    organization_model::ModelCombinationSet::const_iterator cit = combinations.begin();
    for(; cit != combinations.end(); ++cit)
    {
        csp_combinations.insert( toCSP(*cit) );
    }
    return csp_combinations;
}

std::vector<uint32_t> ModelDistribution::toCSP(const organization_model::ModelCombination& combination) const
{
    std::vector<uint32_t> csp_combination;
    organization_model::ModelCombination::const_iterator cit = combination.begin();
    for(; cit != combination.end(); ++cit)
    {
        csp_combination.push_back( systemModelToCSP(*cit) );
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

ModelDistribution::ModelDistribution(const templ::Mission& mission)
    : Gecode::Space()
    , mMission(mission)
    , mSetAssignments()
    , mAsk(mission.getOrganizationModel(), mission.getResources())
{
    mServices = mission.getInvolvedServices();

    // TODO: Check for interval overlaps
    std::set<solvers::temporal::Interval> intervals = mission.getTimeIntervals();
    mIntervals.insert(mIntervals.begin(), intervals.begin(), intervals.end());

    std::set<ObjectVariable::Ptr> variables = mission.getObjectVariables();
    mVariables.insert(mVariables.begin(), variables.begin(), variables.end());

    // Create list of distinct service requirements (based on different time and
    // location)
    for(uint32_t s = 0; s < mServices.size(); ++s)
    {
        for(uint32_t i = 0; i < mIntervals.size(); ++i)
        {
            for(uint32_t v = 0; v < mVariables.size(); ++v)
            {
                FluentTimeService lts(s,i,v);
                mRequirements.push_back(lts);

                // Todo: register requirement for each timeinterval
                mConcurrentRequirements[i].push_back(lts);
            }
        }
    }

    // Compute set of available models
    organization_model::ModelPool modelPool = mission.getResources();
    organization_model::ModelPool::const_iterator cit = modelPool.begin();

    Gecode::IntArgs maximumAvailableResourcesArgs;
    uint32_t index = 0;
    for(; cit != modelPool.end(); ++cit)
    {
        mAvailableModels.push_back(cit->first);
        for(uint32_t count = 0; count < cit->second; ++count)
        {
            maximumAvailableResourcesArgs << index;
        }
        ++index;
    }
    Gecode::IntSet maximumAvailableResources(maximumAvailableResourcesArgs);

    // Compute domain for each requirement and total domain
    organization_model::ModelCombinationSet domainModels;
    std::vector<FluentTimeService>::const_iterator rit = mRequirements.begin();
    for(; rit != mRequirements.end(); ++rit)
    {
        organization_model::ModelCombinationSet combinations = getDomain(*rit);
        mRequirementsDomain[*rit] = combinations;

        std::set_union(domainModels.begin(), domainModels.end(), 
                combinations.begin(), combinations.end(),
                std::inserter(domainModels, domainModels.begin()));
    }
    LOG_DEBUG_S << "Number of model combinations to be considered: " << domainModels.size();
    mDomain.insert(mDomain.begin(), domainModels.begin(), domainModels.end());

    // Start actual formulation into Gecode 

    // The requirements can be fulfilled by model that are part of the domain
    Gecode::IntSet allDomainValues(0, mDomain.size() - 1);

    // SRT_SUB --> subset
    dom(*this, mSetAssignments, Gecode::SRT_SUB, allDomainValues);

    // Domain must not be empty
    // domain set assignement: SRT_NQ --> disequality
    dom(*this, mSetAssignments, Gecode::SRT_NQ, Gecode::IntSet(1,0));

    // Make sure the assignments are unique for concurrent requirements
    for(uint32_t ci = 0; ci < mConcurrentRequirements.size(); ++ci)
    {
        const std::vector<FluentTimeService>& fluents = mConcurrentRequirements[ci];
        std::vector<FluentTimeService>::const_iterator fit = fluents.begin();

        std::vector<int> indexes;
        for(; fit != fluents.end(); ++fit)
        {
            std::vector<FluentTimeService>::const_iterator ftsIt = std::find(mRequirements.begin(), mRequirements.end(), *fit);
            if(ftsIt != mRequirements.end())
            {
                indexes.push_back( ftsIt - mRequirements.begin());
            }
        }

        // If there is just one entry -- no need for computing disjoint relationship
        if(indexes.size() > 1)
        {
            numeric::Combination<int> combination(indexes, 2, numeric::EXACT);
            do {
                std::vector<int> indexes = combination.current();
                // SRT_DISJ --> disjoint
                rel(*this, mSetAssignments[ indexes[0] ], Gecode::SRT_DISJ, mSetAssignments[ indexes[1] ]);
            } while(combination.next());
        }
    }
    // Restrict domains
    for(uint32_t a = 0; a < mRequirements.size(); ++a)
    {
        // Limit the domain of the current restriction to the allowed domain
        // values
        const FluentTimeService& requirement = mRequirements[a];
        const organization_model::ModelCombinationSet& allowedCombinations = mRequirementsDomain[requirement];

        // Fill in allowed domain values
        Gecode::IntArgs args;
        {
            organization_model::ModelCombinationSet::const_iterator dit = allowedCombinations.begin();
            for(; dit != allowedCombinations.end(); ++dit)
            {
                organization_model::ModelCombinationList::const_iterator aIt = std::find(mDomain.begin(), mDomain.end(), *dit);
                if(aIt == mDomain.end())
                {
                    throw std::invalid_argument("templ::solvers::csp::ModelDistribution: domain element"
                            " not found");
                } else {
                    args << static_cast<int>(aIt - mDomain.begin());
                }
            }
        }
        Gecode::IntSet allowedResources(args);
        dom(*this, mSetAssignments[a], Gecode::SRT_SUB, allowedResources);
    }

    //rel(*this, mSetAssignments[0] + mSetAssignments[1], Gecode::SRT_SUB, maximumAvailableResources);
}

ModelDistribution::ModelDistribution(bool share, ModelDistribution& other)
    : Gecode::Space(share, other)
    , mMission(other.mMission)
    , mAsk(other.mAsk)
    , mServices(other.mServices)
    , mIntervals(other.mIntervals)
    , mVariables(other.mVariables)
    , mRequirements(other.mRequirements)
    , mDomain(other.mDomain)
{
    mSetAssignments.update(*this, share, other.mSetAssignments);
}

Gecode::Space* ModelDistribution::copy(bool share)
{
    return new ModelDistribution(share, *this);
}

std::vector<Solution> ModelDistribution::solve(const templ::Mission& mission)
{
    std::vector<Solution> solutions;

    ModelDistribution* distribution = new ModelDistribution(mission);
    ModelDistribution* solvedDistribution = distribution->solve();
    solutions.push_back(solvedDistribution->getSolution());
    delete solvedDistribution;
    solvedDistribution = NULL;

    return solutions;
}

std::string ModelDistribution::toString() const
{
    throw std::runtime_error("ModelDistribution::toString: not implemented");
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
