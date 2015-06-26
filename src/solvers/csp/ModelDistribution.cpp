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
        LOG_WARN_S << "Found solution";
        delete best;
        best = current;

        using namespace organization_model;
        ModelPoolDelta allGblConsumedResources;
        ModelPoolDelta allLubConsumedResources;
        LOG_WARN_S << "Assignment size: " << mSetAssignments.size();
        // Check if resource requirements holds
        for(int i = 0; i < mSetAssignments.size(); ++i)
        {
            LOG_WARN_S << "GET Assign";
            for(Gecode::SetVarGlbValues v(mSetAssignments[i]); v(); ++v)
            {
                // v.val() --> the assigned value
                ModelCombination mc = mDomain[v.val()];
                LOG_WARN_S << "ModelCombination (glb): " << owlapi::model::IRI::toString(mc, true);
                ModelPool consumedResources = OrganizationModel::combination2ModelPool(mc);

                allGblConsumedResources = Algebra::sum(allGblConsumedResources, consumedResources);
            }

            for(Gecode::SetVarLubValues v(mSetAssignments[i]); v(); ++v)
            {
                // v.val() --> the assigned value
                ModelCombination mc = mDomain[v.val()];
                LOG_WARN_S << "ModelCombination (lub): " << owlapi::model::IRI::toString(mc, true);
                ModelPool consumedResources = OrganizationModel::combination2ModelPool(mc);

                allLubConsumedResources = Algebra::sum(allLubConsumedResources, consumedResources);
            }
        }
        ModelPoolDelta diff = Algebra::delta(mMission.getResources(), allLubConsumedResources);
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
    LOG_WARN_S << "Services size: " << services.size() << " " << services.begin()->getModel().toString();

    organization_model::ModelCombinationSet combinations = mAsk.getBoundedResourceSupport(services);

    LOG_WARN_S << "Bounded resources: " << organization_model::OrganizationModel::toString(combinations);
    return combinations;
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
    LOG_INFO_S << "ModelDistribution CSP Problem Construction";
    mMission.prepare();

    {
        using namespace solvers::temporal;
        LOG_WARN_S << mAsk.toString();
        owlapi::model::IRISet services = mMission.getInvolvedServices();
        mServices.insert(mServices.begin(), services.begin(), services.end());

        LOG_DEBUG_S << "Involved services: " << owlapi::model::IRI::toString(mServices, true);

        // TODO: Check for interval overlaps
        std::unordered_set<Interval> intervals = mMission.getTimeIntervals();
        mIntervals.insert(mIntervals.begin(), intervals.begin(), intervals.end());

        std::set<ObjectVariable::Ptr> variables = mMission.getObjectVariables();
        mVariables.insert(mVariables.begin(), variables.begin(), variables.end());

        std::vector<PersistenceCondition::Ptr> conditions = mMission.getPersistenceConditions();
        std::vector<PersistenceCondition::Ptr>::const_iterator cit = conditions.begin();
        for(; cit != conditions.end(); ++cit)
        {
            PersistenceCondition::Ptr p = *cit;

            Interval interval(p->getFromTimePoint(), p->getToTimePoint(), 
                    point_algebra::TimePointComparator(mMission.getTemporalConstraintNetwork()) );

            owlapi::model::IRI serviceModel(p->getStateVariable().getResource());

            ObjectVariable::Ptr objectVariable = boost::dynamic_pointer_cast<ObjectVariable>(p->getValue());

            {
                std::vector<Interval>::const_iterator iit = std::find(mIntervals.begin(), mIntervals.end(), interval);
                if(iit == mIntervals.end())
                {
                    throw std::runtime_error("Could not find interval");
                }

                owlapi::model::IRIList::const_iterator sit = std::find(mServices.begin(), mServices.end(), serviceModel);
                if(sit == mServices.end())
                {
                    throw std::runtime_error("Could not find service");
                }

                std::vector<ObjectVariable::Ptr>::const_iterator oit = std::find(mVariables.begin(), mVariables.end(), objectVariable);
                if(oit == mVariables.end())
                {
                    throw std::runtime_error("Could not find variable");
                }

                uint32_t timeIndex = iit - mIntervals.begin();
                FluentTimeService lts((int) (sit - mServices.begin()),
                        timeIndex,
                        (int) (oit - mVariables.begin()));

                mRequirements.push_back(lts);
                LOG_DEBUG_S << lts.toString();

                // Todo: register requirement for each timeinterval
                mTimeIndexedRequirements[timeIndex].push_back(lts);
            }
        }
    }

    // Create list of distinct service requirements (based on different time and
    // location)
    //for(uint32_t s = 0; s < mServices.size(); ++s)
    //{
    //    for(uint32_t i = 0; i < mIntervals.size(); ++i)
    //    {
    //        for(uint32_t v = 0; v < mVariables.size(); ++v)
    //        {
    //            FluentTimeService lts(s,i,v);
    //            mRequirements.push_back(lts);

    //            LOG_DEBUG_S << "FluentTimeService: "
    //                << " service #" << s
    //                << " interval #" << i
    //                << " variable #" << v;

    //            // Todo: register requirement for each timeinterval
    //            mConcurrentRequirements[i].push_back(lts);
    //        }
    //    }
    //}

    // Compute set of available models
    organization_model::ModelPool modelPool = mission.getResources();
    organization_model::ModelPool::const_iterator mit = modelPool.begin();

    Gecode::IntArgs maximumAvailableResourcesArgs;
    uint32_t index = 0;
    for(; mit != modelPool.end(); ++mit)
    {
        mAvailableModels.push_back(mit->first);
        for(uint32_t count = 0; count < mit->second; ++count)
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
        LOG_WARN_S << "Domain size: " << combinations.size();
        mRequirementsDomain[*rit] = combinations;

        std::set_union(domainModels.begin(), domainModels.end(), 
                combinations.begin(), combinations.end(),
                std::inserter(domainModels, domainModels.begin()));
    }
    LOG_WARN_S << "Number of model combinations to be considered: " << domainModels.size();
    mDomain.insert(mDomain.begin(), domainModels.begin(), domainModels.end());

    // Start actual formulation into Gecode 

    // The requirements can be fulfilled by model that are part of the domain
    Gecode::IntSet allDomainValues(0, mDomain.size() - 1);
    LOG_WARN_S << "Domain size: " << mDomain.size();

    Gecode::SetVarArray setAssignements(*this, mRequirements.size());
    mSetAssignments.update(*this, false, setAssignements);

    // SRT_SUB --> subset
    dom(*this, mSetAssignments, Gecode::SRT_SUB, allDomainValues);

    // Domain must not be empty
    // domain set assignement: SRT_NQ --> disequality
    dom(*this, mSetAssignments, Gecode::SRT_NQ, Gecode::IntSet(1,0));

    std::vector<size_t> timeIndexes;
    for(size_t ti = 0; ti < mIntervals.size(); ++ti)
    {
        timeIndexes.push_back(ti);
    }

    LOG_WARN_S << "TimeIndices available: " << timeIndexes.size();
    if(timeIndexes.size() == 1)
    {
        std::vector<size_t> concurrentFluentIndexes;
        LOG_DEBUG_S << "Concurrent fluent indexes";
        for(size_t fi = 0; fi < mRequirements.size(); ++fi)
        {
            concurrentFluentIndexes.push_back(fi);
            LOG_DEBUG_S << "    #" << fi;
        }

        // If there is just one entry -- no need for computing disjoint relationship
        if(concurrentFluentIndexes.size() > 1)
        {
            numeric::Combination<size_t> combination(concurrentFluentIndexes, 2, numeric::EXACT);
            do {
                std::vector<size_t> fluentIndexes = combination.current();
                // SRT_DISJ --> disjoint
                rel(*this, mSetAssignments[ fluentIndexes[0] ], Gecode::SRT_DISJ, mSetAssignments[ fluentIndexes[1] ]);
            } while(combination.next());
        }

    } else {
        numeric::Combination<size_t> intervalCombination(timeIndexes, 2, numeric::EXACT);
        do {
            using namespace templ::solvers::temporal;

            std::vector<size_t> timeIndex = intervalCombination.current();

            const Interval& i0 = mIntervals[ timeIndex[0] ];
            const Interval& i1 = mIntervals[ timeIndex[1] ];

            if(!i0.overlaps(i1))
            {
                // no timeoverlap, so we do not care
                continue;
            } else {
                LOG_DEBUG_S << "Intervals overlap: " << std::endl
                    << i0.toString() << std::endl
                    << i1.toString();
            }

            // Get indexes of requirements
            const std::vector<FluentTimeService>& fluents0 = mTimeIndexedRequirements[ timeIndex[0] ];
            const std::vector<FluentTimeService>& fluents1 = mTimeIndexedRequirements[ timeIndex[1] ];

            std::set<size_t> concurrentFluentIndexesSet;
            {
                std::vector<FluentTimeService>::const_iterator fit = fluents0.begin();
                for(; fit != fluents0.end(); ++fit)
                {
                    concurrentFluentIndexesSet.insert( getFluentIndex(*fit) );
                }
            }
            {
                std::vector<FluentTimeService>::const_iterator fit = fluents1.begin();
                for(; fit != fluents1.end(); ++fit)
                {
                    concurrentFluentIndexesSet.insert( getFluentIndex(*fit) );
                }
            }

            std::vector<size_t> concurrentFluentIndexes(concurrentFluentIndexesSet.begin(),
                    concurrentFluentIndexesSet.end());

            // If there is just one entry -- no need for computing disjoint relationship
            if(concurrentFluentIndexes.size() > 1)
            {
                numeric::Combination<size_t> combination(concurrentFluentIndexes, 2, numeric::EXACT);
                do {
                    std::vector<size_t> fluentIndexes = combination.current();
                    LOG_DEBUG_S << "Concurrent fluents: " << std::endl
                        << mRequirements[ fluentIndexes[0] ].toString() << std::endl
                        << mRequirements[ fluentIndexes[1] ].toString() << std::endl;

                    // SRT_DISJ --> disjoint
                    rel(*this, mSetAssignments[ fluentIndexes[0] ], Gecode::SRT_DISJ, mSetAssignments[ fluentIndexes[1] ]);
                    // that will not work here, e.g. when
                    //  2 Sherpa exist, and one is assigned to each one, then
                    //  this condition still fails
                    //  probably better:
                    //      get ALL overlapping requirements, i.e. not just the
                    //      mutual ones and make sure their subset is < then the
                    //      set of available resource --> which actually should
                    //      have been done in the first place -- try again,
                    //      fail again, fail harder ;)

                } while(combination.next());
            }
        } while(intervalCombination.next());
    }

    //// Make sure the assignments are unique for concurrent requirements
    //for(uint32_t ci = 0; ci < mTimeIndexedRequirements.size(); ++ci)
    //{
    //    const std::vector<FluentTimeService>& fluents = mConcurrentRequirements[ci];
    //    std::vector<FluentTimeService>::const_iterator fit = fluents.begin();

    //    std::vector<int> indexes;
    //    LOG_DEBUG_S << "Concurrent requirements: ";
    //    for(; fit != fluents.end(); ++fit)
    //    {
    //        indexes.push_back( getFluentIndex(*fit) );
    //    }

    //    // If there is just one entry -- no need for computing disjoint relationship
    //    if(indexes.size() > 1)
    //    {
    //        numeric::Combination<int> combination(indexes, 2, numeric::EXACT);
    //        do {
    //            std::vector<int> indexes = combination.current();
    //            // SRT_DISJ --> disjoint
    //            rel(*this, mSetAssignments[ indexes[0] ], Gecode::SRT_DISJ, mSetAssignments[ indexes[1] ]);
    //        } while(combination.next());
    //    }
    //}

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

size_t ModelDistribution::getFluentIndex(const FluentTimeService& fluent) const
{
    std::vector<FluentTimeService>::const_iterator ftsIt = std::find(mRequirements.begin(), mRequirements.end(), fluent);
    if(ftsIt != mRequirements.end())
    {
        int index = ftsIt - mRequirements.begin();
        assert(index >= 0);
        return (size_t) index;
    }

    throw std::runtime_error("templ::solvers::csp::ModelDistribution::getFluentIndex: could not find fluent index for '" + fluent.toString() + "'");
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
