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

ModelDistribution::ModelDistribution(const templ::Mission& mission)
    : Gecode::Space()
{
}

ModelDistribution::ModelDistribution(bool share, ModelDistribution& other)
    : Gecode::Space(share, other)
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
