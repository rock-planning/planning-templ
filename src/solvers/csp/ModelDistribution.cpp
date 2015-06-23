#include "ModelDistribution.hpp"

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

        current->getSolution();
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
            }
        }
    }

    // Update list of available models
    organization_model::ModelPool modelPool = mission.getResources();
    organization_model::ModelPool::const_iterator cit = modelPool.begin();
    for(; cit != modelPool.end(); ++cit)
    {
        mAvailableModels.push_back(cit->first);
    }
}

ModelDistribution::ModelDistribution(bool share, ModelDistribution& other)
    : Gecode::Space(share, other)
    , mAsk(other.mAsk)
{
}

Gecode::Space* ModelDistribution::copy(bool share)
{
    return new ModelDistribution(share, *this);
}

std::vector<Solution> ModelDistribution::solve(const templ::Mission& mission)
{
    std::vector<Solution> solutions;

    return solutions;
}

std::string ModelDistribution::toString() const
{
    throw std::runtime_error("ModelDistribution::toString: not implemented");
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
