#include "RoleDistribution.hpp"

#include <gecode/minimodel.hh>
#include <gecode/set.hh>
#include <gecode/search.hh>
#include <base/Logging.hpp>

namespace templ {
namespace solvers {
namespace csp {

RoleDistribution::RoleDistribution(const Mission& mission, const ModelDistribution::Solution& modelDistribution)
    : Gecode::Space()
    , mRoleUsage(*this, /*width --> col */ mission.getRoles().size()* /*height --> row*/ modelDistribution.size(), 0, 1) // Domain 0,1 to represent activation
    , mRoles(mission.getRoles())
    , mIntervals(mission.getTimeIntervals().begin(), mission.getTimeIntervals().end())
    , mAvailableModels(mission.getModels())
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ modelDistribution.size());

    const owlapi::model::IRIList& models = mission.getModels();
    const organization_model::ModelPool& resources = mission.getResources();

    // foreach FluentTimeService
    //     same role types -> sum <= modelbound given by solution
    ModelDistribution::Solution::const_iterator cit = modelDistribution.begin();
    size_t column = 0;
    for(; cit != modelDistribution.end(); ++cit, ++column)
    {
        const FluentTimeService& fts = cit->first;
        const organization_model::ModelPool& solutionPool = cit->second;

        // build initial requirement list
        mRequirements.push_back(fts);

        // Set limits per model type
        size_t index = 0;
        for(size_t i = 0; i < models.size(); ++i)
        {
            Gecode::IntVarArgs args;
            const owlapi::model::IRI& currentModel = models[i];

            organization_model::ModelPool::const_iterator mit = resources.find(currentModel);
            if(mit == resources.end())
            {
                throw std::runtime_error("templ::solvers::csp::RoleDistribution "
                        " could not find model: '" + currentModel.toString() + "'");
            }
            size_t modelBound = mit->second;
            for(size_t t = 0; t < modelBound; ++t)
            {
                Gecode::IntVar v = roleDistribution(index, column);
                args << v;

                ++index;
            }

            mit = solutionPool.find(currentModel);
            if(mit == resources.end())
            {
                throw std::runtime_error("templ::solvers::csp::RoleDistribution "
                        " could not find model: '" + currentModel.toString() + "'");
            }
            size_t solutionModelBound = mit->second;

            rel(*this, sum(args) == solutionModelBound);
        }
    }

    assert(mRequirements.size() == modelDistribution.size());

    {
        // Set of available models: mModelPool
        // Make sure the assignments are within resource bounds for concurrent requirements
        std::vector< std::vector<FluentTimeService> > concurrentRequirements = FluentTimeService::getConcurrent(mRequirements, mIntervals);

        std::vector< std::vector<FluentTimeService> >::const_iterator cit = concurrentRequirements.begin();
        if(concurrentRequirements.empty())
        {
            LOG_WARN_S << "No concurrent requirements found";
        } else {
            for(; cit != concurrentRequirements.end(); ++cit)
            {
                LOG_DEBUG_S << "Concurrent roles requirements: " << mRoles.size();
                const std::vector<FluentTimeService>& concurrentFluents = *cit;

                for(size_t roleIndex = 0; roleIndex < mRoles.size(); ++roleIndex)
                {
                    Gecode::IntVarArgs args;

                    std::vector<FluentTimeService>::const_iterator fit = concurrentFluents.begin();
                    for(; fit != concurrentFluents.end(); ++fit)
                    {
                        size_t row = getFluentIndex(*fit);
                        LOG_DEBUG_S << "    index: " << roleIndex << "/" << row;
                        Gecode::IntVar v = roleDistribution(roleIndex, row);
                        args << v;
                    }
                    rel(*this, sum(args) <= 1);
                }
            }
        }
    }

    branch(*this, mRoleUsage, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_SPLIT_MIN());
    branch(*this, mRoleUsage, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_SPLIT_MIN());
    branch(*this, mRoleUsage, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_SPLIT_MIN());
}

RoleDistribution::RoleDistribution(bool share, RoleDistribution& other)
    : Gecode::Space(share, other)
    , mRoles(other.mRoles)
    , mRequirements(other.mRequirements)
    , mIntervals(other.mIntervals)
{
    mRoleUsage.update(*this, share, other.mRoleUsage);

}

Gecode::Space* RoleDistribution::copy(bool share)
{
    return new RoleDistribution(share, *this);
}

size_t RoleDistribution::getFluentIndex(const FluentTimeService& fluent) const
{
    std::vector<FluentTimeService>::const_iterator ftsIt = std::find(mRequirements.begin(), mRequirements.end(), fluent);
    if(ftsIt != mRequirements.end())
    {
        int index = ftsIt - mRequirements.begin();
        assert(index >= 0);
        return (size_t) index;
    }

    throw std::runtime_error("templ::solvers::csp::RoleDistribution::getFluentIndex: could not find fluent index for '" + fluent.toString() + "'");
}

RoleDistribution::SolutionList RoleDistribution::solve(const Mission& _mission, const ModelDistribution::Solution& modelDistribution)
{
    SolutionList solutions;

    Mission mission = _mission;
    mission.prepare();

    RoleDistribution* distribution = new RoleDistribution(mission, modelDistribution);
    RoleDistribution* solvedDistribution = NULL;
    {
        Gecode::BAB<RoleDistribution> searchEngine(distribution);
        //Gecode::DFS<ModelDistribution> searchEngine(this);

        RoleDistribution* best = NULL;
        while(RoleDistribution* current = searchEngine.next())
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

RoleDistribution::Solution RoleDistribution::getSolution() const
{
    Solution solution;

    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mRequirements.size());

    // Check if resource requirements holds
    for(size_t i = 0; i < mRequirements.size(); ++i)
    {
        Role::List roles;
        for(size_t r = 0; r < mRoles.size(); ++r)
        {
            Gecode::IntVar var = roleDistribution(r, i);
            if(!var.assigned())
            {
                throw std::runtime_error("templ::solvers::csp::RoleDistribution::getSolution: value has not been assigned");
            }

            Gecode::IntVarValues v( var );

            if( v.val() == 1 )
            {
                roles.push_back( mRoles[r] );
            }
        }

        solution[ mRequirements[i] ] = roles;
    }

    return solution;
}


std::string RoleDistribution::toString() const
{
    std::stringstream ss;
    ss << "ModelDistribution: #" << std::endl;
    ss << "    role usage: " << mRoleUsage;
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const RoleDistribution::Solution& solution)
{
    RoleDistribution::Solution::const_iterator cit = solution.begin();
    size_t count = 0;
    os << "Solution" << std::endl;
    for(; cit != solution.end(); ++cit)
    {
        const FluentTimeService& fts = cit->first;
        os << "--- requirement #" << count++ << std::endl;
        os << fts.toString() << std::endl;

        const Role::List& roles = cit->second;
        os << Role::toString(roles) << std::endl;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const RoleDistribution::SolutionList& solutions)
{
    RoleDistribution::SolutionList::const_iterator cit = solutions.begin();
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
