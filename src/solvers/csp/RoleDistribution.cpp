#include <numeric/Combinatorics.hpp>
#include <gecode/minimodel.hh>
#include <gecode/set.hh>
#include <gecode/search.hh>
#include <base-logging/Logging.hpp>

#include "RoleDistribution.hpp"
#include "MissionConstraints.hpp"


namespace templ {
namespace solvers {
namespace csp {


RoleDistribution::SearchState::SearchState(const ModelDistribution::SearchState& modelSearchState)
    : mpInitialState(NULL)
    , mpSearchEngine(NULL)
    , mType(OPEN)
{
    mpInitialState = RoleDistribution::Ptr(new RoleDistribution(modelSearchState.getMission(), modelSearchState.getModelDistributionSolution()) );
    mpSearchEngine = RoleDistribution::BABSearchEnginePtr(new Gecode::BAB<RoleDistribution>(mpInitialState.get()));
}

RoleDistribution::SearchState::SearchState(const RoleDistribution::Ptr& roleDistribution,
        const RoleDistribution::BABSearchEnginePtr& searchEngine)
    : mpInitialState(roleDistribution)
    , mpSearchEngine(searchEngine)
    , mType(OPEN)
{
    assert(mpInitialState);
    if(!mpSearchEngine)
    {
        mpSearchEngine = RoleDistribution::BABSearchEnginePtr(new Gecode::BAB<RoleDistribution>(mpInitialState.get()));
    }
}

RoleDistribution::SearchState RoleDistribution::SearchState::next() const
{
    if(!getInitialState())
    {
        throw std::runtime_error("templ::solvers::csp::RoleDistribution::SearchState::next: "
                " next() called on an unitialized search state");
    }

    SearchState searchState(mpInitialState, mpSearchEngine);
    RoleDistribution* solvedDistribution = mpSearchEngine->next();
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


RoleDistribution::RoleDistribution(const Mission::Ptr& mission, const ModelDistribution::ModelDistributionSolution& modelDistribution)
    : Gecode::Space()
    , mRoleUsage(*this, /*width --> col */ mission->getRoles().size()* /*height --> row*/ modelDistribution.size(), 0, 1) // Domain 0,1 to represent activation
    , mRoles(mission->getRoles())
    , mIntervals(mission->getTimeIntervals().begin(), mission->getTimeIntervals().end())
{
    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ modelDistribution.size());

    const owlapi::model::IRIList& availableModels = mission->getModels();
    const organization_model::ModelPool& resources = mission->getAvailableResources();

    // foreach FluentTimeResource
    //     same role types -> sum <= modelbound given by solution
    ModelDistribution::ModelDistributionSolution::const_iterator cit = modelDistribution.begin();
    size_t column = 0;
    for(; cit != modelDistribution.end(); ++cit, ++column)
    {
        const FluentTimeResource& fts = cit->first;
        const organization_model::ModelPool& solutionPool = cit->second;

        // build initial requirement list
        mRequirements.push_back(fts);

        // Set limits per model type
        size_t index = 0;
        for(size_t i = 0; i < availableModels.size(); ++i)
        {
            Gecode::IntVarArgs args;
            const owlapi::model::IRI& currentModel = availableModels[i];

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
        std::vector< std::vector<FluentTimeResource> > concurrentRequirements = FluentTimeResource::getConcurrent(mRequirements, mIntervals);

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
                    rel(*this, sum(args) <= 1);
                }
            }
        } else {
            LOG_DEBUG_S << "No concurrent requirements found";
        }
    }

    // Avoid computation of solutions that are redunant
    // Gecode documentation says however in 8.10.2 that "Symmetry breaking by
    // LDSB is not guaranteed to be complete. That is, a search may still return
    // two distinct solutions that are symmetric."
    //
    Gecode::Symmetries symmetries;
    // define interchangeable columns for roles of the same model type
    owlapi::model::IRIList::const_iterator ait = availableModels.begin();
    for(; ait != availableModels.end(); ++ait)
    {
        const owlapi::model::IRI& currentModel = *ait;
        LOG_INFO_S << "Starting symmetry column for model: " << currentModel.toString();
        Gecode::IntVarArgs sameModelColumns;
        for(int c = 0; c < roleDistribution.width(); ++c)
        {
            if( mRoles[c].getModel() == currentModel)
            {
                LOG_INFO_S << "Adding column of " << mRoles[c].toString() << " for symmetry";
                sameModelColumns << roleDistribution.col(c);
            }
        }
        symmetries << VariableSequenceSymmetry(sameModelColumns, roleDistribution.height());
    }

    branch(*this, mRoleUsage, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_MIN(), symmetries);
    branch(*this, mRoleUsage, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_MIN(), symmetries);
    branch(*this, mRoleUsage, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_MIN(), symmetries);
}

RoleDistribution::RoleDistribution(RoleDistribution& other)
    : Gecode::Space(other)
    , mRoles(other.mRoles)
    , mRequirements(other.mRequirements)
    , mIntervals(other.mIntervals)
{
    mRoleUsage.update(*this, other.mRoleUsage);

}

Gecode::Space* RoleDistribution::copy()
{
    return new RoleDistribution(*this);
}

size_t RoleDistribution::getFluentIndex(const FluentTimeResource& fluent) const
{
    return FluentTimeResource::getIndex(mRequirements, fluent);
}

RoleDistribution::SolutionList RoleDistribution::solve(const Mission::Ptr& mission, const ModelDistribution::ModelDistributionSolution& modelDistribution)
{
    SolutionList solutions;

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
                throw std::runtime_error("templ::solvers::csp::RoleDistribution::getSolution: value has not been assigned for role: '" + mRoles[r].toString() + "'");
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

void RoleDistribution::allDistinct(const FluentTimeResource& fts0, const FluentTimeResource& fts1, const owlapi::model::IRI& roleModel)
{
    MissionConstraints::allDistinct(*this, mRoleUsage, mRoles, mRequirements,
            fts0, fts1,
            roleModel);
}

void RoleDistribution::minDistinct(const FluentTimeResource& fts0, const FluentTimeResource& fts1, const owlapi::model::IRI& roleModel, uint32_t minDistinctRoles)
{
    MissionConstraints::minDistinct(*this, mRoleUsage, mRoles, mRequirements,
            fts0, fts1,
            roleModel, minDistinctRoles);
}

void RoleDistribution::addDistinct(const FluentTimeResource& fts0, const FluentTimeResource& fts1, const owlapi::model::IRI& roleModel, uint32_t additional, const Solution& solution)
{
    // TODO: replace with MissionConstraints implementation, but require to
    // refactor the solution argument
    //

    Gecode::Matrix<Gecode::IntVarArray> roleDistribution(mRoleUsage, /*width --> col*/ mRoles.size(), /*height --> row*/ mRequirements.size());

    // Adding this constraint will only work to an already once solved instance
    // of the problem
    std::set<Role> uniqueRoles;
    {
        Solution::const_iterator sit = solution.find(fts0);
        if(sit == solution.end())
        {
            throw std::runtime_error("templ::solvers::csp::RoleDistribution: the given fluent-time-resource is not part of the solution: "
                    + fts0.toString());
        }

        const Role::List& roles = sit->second;
        Role::List::const_iterator rit = roles.begin();
        for(; rit != roles.end(); ++rit)
        {
            const Role& role = *rit;
            if(role.getModel() == roleModel)
            {
                uniqueRoles.insert(role);
            }
        }
    }
    {
        Solution::const_iterator sit = solution.find(fts1);
        if(sit == solution.end())
        {
            throw std::runtime_error("templ::solvers::csp::RoleDistribution: the given fluent-time-resource is not part of the solution"
                    + fts1.toString());
        }

        const Role::List& roles = sit->second;
        Role::List::const_iterator rit = roles.begin();
        for(; rit != roles.end(); ++rit)
        {
            const Role& role = *rit;
            if(role.getModel() == roleModel)
            {
                uniqueRoles.insert(role);
            }
        }
    }
    size_t numberOfUniqueRoles = uniqueRoles.size();
    LOG_INFO_S << "Previous number of unique roles: " << numberOfUniqueRoles << " -- should be increased with " << additional;
    RoleDistribution::minDistinct(fts0, fts1, roleModel, numberOfUniqueRoles + additional);
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ

namespace std {

std::ostream& operator<<(std::ostream& os, const templ::solvers::csp::RoleDistribution::Solution& solution)
{
    using namespace templ;
    using namespace templ::solvers;
    using namespace templ::solvers::csp;

    RoleDistribution::Solution::const_iterator cit = solution.begin();
    size_t count = 0;
    os << "Solution" << std::endl;
    for(; cit != solution.end(); ++cit)
    {
        const FluentTimeResource& fts = cit->first;
        os << "--- requirement #" << count++ << std::endl;
        os << fts.toString() << std::endl;

        const Role::List& roles = cit->second;
        os << Role::toString(roles) << std::endl;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const templ::solvers::csp::RoleDistribution::SolutionList& solutions)
{
    templ::solvers::csp::RoleDistribution::SolutionList::const_iterator cit = solutions.begin();
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

} // end namespace std
