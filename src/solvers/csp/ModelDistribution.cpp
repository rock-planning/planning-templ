#include "ModelDistribution.hpp"
#include <base/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include <gecode/minimodel.hh>

#include <gecode/gist.hh>
#include <organization_model/Algebra.hpp>

namespace templ {
namespace solvers {
namespace csp {

ModelDistribution* ModelDistribution::solve()
{
    throw std::runtime_error("Should not call solve");
//    Gecode::BAB<ModelDistribution> searchEngine(this);
//    //Gecode::DFS<ModelDistribution> searchEngine(this);
//
//    ModelDistribution* best = NULL;
//    while(ModelDistribution* current = searchEngine.next())
//    {
//        delete best;
//        best = current;
//
//        using namespace organization_model;
//        Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, mMission.getResources().size(), mRequirements.size());
//
//        LOG_INFO_S << "Solution found:" << toString();
////        // Check if resource requirements holds
////        for(size_t i = 0; i < mRequirements.size(); ++i)
////        {
////            LOG_DEBUG_S << "assignment for:"
////                << mRequirements[i].toString()
////                << " available resources: " << mMission.getResources().size();
////
////            for(size_t mi = 0; mi < mMission.getResources().size(); ++mi)
////            {
////                Gecode::IntVar var = resourceDistribution(mi, i);
////                Gecode::IntVarValues v( var );
////                LOG_DEBUG_S << "    "
////                    << mAvailableModels[mi].toString()
////                    << ": #" << v.val();
////            }
////        }
//        //break;
//    }
//
//    if(best == NULL)
//    {
//        throw std::runtime_error("templ::solvers::csp::ModelDistribution::solve: no solution found");
//    }
//
//    return best;
}

Solution ModelDistribution::getSolution() const
{
    Solution solution;
    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, mModelPool.size(), mRequirements.size());

    // Check if resource requirements holds
    for(size_t i = 0; i < mRequirements.size(); ++i)
    {
        organization_model::ModelPool modelPool;
        for(size_t mi = 0; mi < mMission.getResources().size(); ++mi)
        {
            Gecode::IntVar var = resourceDistribution(mi, i);
            if(!var.assigned())
            {
                throw std::runtime_error("templ::solvers::csp::ModelDistribution::getSolution: value has not been assigned");
            }

            Gecode::IntVarValues v( var );

            modelPool[ mAvailableModels[mi] ] = v.val();
        }

        solution[ mRequirements[i] ] = modelPool;
    }

    return solution;
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
    // return index of model and count per model
    std::vector<uint32_t> csp_combination;
    for(size_t i = 0; i < mModelPool.size(); ++i)
    {
        csp_combination.push_back(0);
    }

    organization_model::ModelCombination::const_iterator cit = combination.begin();
    for(; cit != combination.end(); ++cit)
    {
        uint32_t index = systemModelToCSP(*cit);
        csp_combination[index]++;
    }
    LOG_WARN_S << "TO CSP: of " << combination << " returns size: " << csp_combination.size();
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
    , mModelPool(mission.getResources())
    , mAsk(mission.getOrganizationModel(), mission.getResources())
    , mServices(mission.getInvolvedServices().begin(), mission.getInvolvedServices().end())
    , mIntervals(mission.getTimeIntervals().begin(), mission.getTimeIntervals().end())
    , mVariables(mission.getObjectVariables().begin(), mission.getObjectVariables().end())
    , mRequirements(getRequirements())
    , mModelUsage(*this, /*# of models*/ mission.getResources().size()*
            /*# of fluent time services*/mRequirements.size(), 0, getMaxResourceCount(mModelPool))
{
    LOG_INFO_S << "ModelDistribution CSP Problem Construction" << std::endl
    << "    services: " << mServices << std::endl
    << "    intervals: " << mIntervals.size() << std::endl;

    organization_model::ModelPool::const_iterator mit = mModelPool.begin();
    for(; mit != mModelPool.end(); ++mit)
    {
        mAvailableModels.push_back(mit->first);
    }

    //Gecode::IntArgs x = Gecode::IntArgs::create(3,2,0);
    //Gecode::IntArgs y = Gecode::IntArgs::create(2,2,0);
    //Gecode::IntArgs z = Gecode::IntArgs::create(2,2,0);
    //LOG_DEBUG_S << "Intargs: " << x;
    //LOG_DEBUG_S << "Intargs: " << Gecode::IntSet( x );

    ////rel(*this, x + y, Gecode::SRT_SUB, z)

    Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, /*width --> col*/ mission.getResources().size(), /*height --> row*/ mRequirements.size());
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
    // tupleSet.add( Gecode::IntArgs::create( mission.getResources().size(), 1, 0) );
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
    // Part (A)
    {
        using namespace solvers::temporal;
        LOG_INFO_S << mAsk.toString();
        LOG_DEBUG_S << "Involved services: " << owlapi::model::IRI::toString(mServices, true);

        // TODO Check for time interval overlaps
        {
            std::vector<FluentTimeService>::const_iterator fit = mRequirements.begin();
            for(; fit != mRequirements.end(); ++fit)
            {
                const FluentTimeService& fts = *fit;
                LOG_DEBUG_S << "(A) Define requirement: " << fts.toString();

                // row: index of requirement
                // col: index of model type
                size_t requirementIndex = fit - mRequirements.begin();
                for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
                {
                    Gecode::IntVar v = resourceDistribution(mi, requirementIndex);
                    // default min requirement is 0
                    rel(*this, v, Gecode::IRT_GQ, 0);
                    // setting the upper bound for this model and this service
                    LOG_DEBUG_S << "requirement: " << requirementIndex
                        << ", model: " << mi
                        << " IRT_GQ 0 IRT_LQ: " << mModelPool[ mAvailableModels[mi] ];
                    rel(*this, v, Gecode::IRT_LQ, mModelPool[ mAvailableModels[mi] ]);

                }

                // Extensional constraints, i.e. specifying the allowed
                // combinations
                organization_model::ModelCombinationSet allowedCombinations = getDomain(fts);
                Gecode::TupleSet tupleSet = toTupleSet(allowedCombinations);
                tupleSet.finalize();
                //LOG_DEBUG_S << "TupleSet: " << tupleSet.size();
                extensional(*this, resourceDistribution.row(requirementIndex), tupleSet);

                // there can be no empty assignment for a service
                rel(*this, sum( resourceDistribution.row(requirementIndex) ) > 0);

                // This can be equivalently modelled using a linear constraint
                // Gecode::IntArgs c(mAvailableModels.size());
                // for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
                //    c[mi] = 1;
                // linear(*this, c, resourceDistribution.row(requirementIndex), Gecode::IRT_GR, 0);
            }
        }
    }
    // Part (B) General resource constraints
    //     - identify overlapping fts, limit resources for these (TODO: better
    //     indentification of overlapping requirements)
    {
        // Set of available models: mModelPool
        // Make sure the assignments are within resource bounds for concurrent requirements
        std::vector< std::vector<FluentTimeService> > concurrentRequirements = getConcurrentRequirements();

        std::vector< std::vector<FluentTimeService> >::const_iterator cit = concurrentRequirements.begin();
        for(; cit != concurrentRequirements.end(); ++cit)
        {
            LOG_DEBUG_S << "Concurrent requirements";
            const std::vector<FluentTimeService>& concurrentFluents = *cit;
            for(size_t mi = 0; mi < mAvailableModels.size(); ++mi)
            {
                LOG_DEBUG_S << "    model: " << mAvailableModels[ mi ].toString();
                Gecode::IntVarArgs args;

                std::vector<FluentTimeService>::const_iterator fit = concurrentFluents.begin();
                for(; fit != concurrentFluents.end(); ++fit)
                {
                    Gecode::IntVar v = resourceDistribution(mi, getFluentIndex(*fit));
                    args << v;

                    LOG_DEBUG_S << "    index: " << mi << "/" << getFluentIndex(*fit);
                }

                uint32_t maxCardinality = mModelPool[ mAvailableModels[mi] ];
                LOG_DEBUG_S << "General resource usage constraint: " << std::endl
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
    , mMission(other.mMission)
    , mModelPool(other.mModelPool)
    , mAsk(other.mAsk)
    , mServices(other.mServices)
    , mIntervals(other.mIntervals)
    , mVariables(other.mVariables)
    , mRequirements(other.mRequirements)
    , mDomain(other.mDomain)
    , mAvailableModels(other.mAvailableModels)
{
    LOG_DEBUG_S << "Copy construct";
    mModelUsage.update(*this, share, other.mModelUsage);
}

Gecode::Space* ModelDistribution::copy(bool share)
{
    return new ModelDistribution(share, *this);
}

std::vector<Solution> ModelDistribution::solve(const templ::Mission& _mission)
{
    SolutionList solutions;

    Mission mission = _mission;
    mission.prepare();
    ModelDistribution* distribution = new ModelDistribution(mission);
    ModelDistribution* solvedDistribution = NULL;
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
            throw std::runtime_error("templ::solvers::csp::ModelDistribution::solve: no solution found");
        }
    }
    delete solvedDistribution;
    solvedDistribution = NULL;

    return solutions;
}

Gecode::TupleSet ModelDistribution::toTupleSet(const organization_model::ModelCombinationSet& combinations) const
{
    Gecode::TupleSet tupleSet;

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

    return tupleSet;
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

std::vector<FluentTimeService> ModelDistribution::getRequirements() const
{
    std::vector<FluentTimeService> requirements;

    using namespace templ::solvers::temporal;
    const std::vector<PersistenceCondition::Ptr>& conditions = mMission.getPersistenceConditions();
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
                LOG_WARN_S << "Size of intervals: " << mIntervals.size();
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

            requirements.push_back(lts);
            LOG_DEBUG_S << lts.toString();
        }
    }
    return requirements;
}

std::vector< std::vector<FluentTimeService> > ModelDistribution::getConcurrentRequirements() const
{
    size_t requirementsCount = mRequirements.size();
    std::vector< std::vector<FluentTimeService> > concurrentRequirements;

    for(size_t i = 0; i < requirementsCount; ++i)
    {
        std::vector<FluentTimeService> concurrentFts;
        const FluentTimeService& iFts = mRequirements.at(i);

        concurrentFts.push_back(iFts);

        for(size_t a = 0; a < requirementsCount; ++a)
        {
            if(a == i)
            {
                continue;
            }

            const FluentTimeService& aFts = mRequirements.at(a);

            using namespace templ::solvers::temporal;
            Interval iInterval = mIntervals[iFts.time];
            Interval aInterval = mIntervals[aFts.time];


            if(iInterval.overlaps(aInterval))
            {
                LOG_DEBUG_S << "    " << iFts.toString();
                concurrentFts.push_back(aFts);
            }
        }

        if(concurrentFts.size() > 1)
        {
            concurrentRequirements.push_back(concurrentFts);
        }
    }

    return concurrentRequirements;
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

    //Gecode::Matrix<Gecode::IntVarArray> resourceDistribution(mModelUsage, mModelPool.size(), mRequirements.size());

    //// Check if resource requirements holds
    //for(size_t i = 0; i < mRequirements.size(); ++i)
    //{
    //    ss << "assignment for:"
    //        << mRequirements[i].toString()
    //        << " available resources: " << mMission.getResources().size()
    //        << std::endl;

    //    for(size_t mi = 0; mi < mMission.getResources().size(); ++mi)
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


std::ostream& operator<<(std::ostream& os, const Solution& solution)
{
    Solution::const_iterator cit = solution.begin();
    size_t count = 0;
    os << "Solution" << std::endl;
    for(; cit != solution.end(); ++cit)
    {
        const FluentTimeService& fts = cit->first;
        os << "--- requirement #" << count++ << std::endl;
        os << fts.toString() << std::endl;
        os << organization_model::ModelPoolDelta(cit->second).toString() << std::endl;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const SolutionList& solutions)
{
    SolutionList::const_iterator cit = solutions.begin();
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

} // end namespace csp
} // end namespace solvers
} // end namespace templ
