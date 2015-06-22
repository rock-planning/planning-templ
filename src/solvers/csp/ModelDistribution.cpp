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
        throw std::runtime_error(__FUNCTION__ + ": no solution found");
    }

    return best;
}

ModelDistribution::ModelDistribution(const temp::Mission& mission)
{
}

ModelDistribution::ModelDistribution(bool share, ModelDistribution& s)
{
}

virtual Gecode::Space* ModelDistribution::copy(bool share)
{
}

std::vector<Solution> ModelDistribution::solve(const templ::Mision& mission)
{
}

std::string ModelDistribution::toString() const
{
    throw std::runtime_error(__FUNCTION__ + ": not implemented");
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
