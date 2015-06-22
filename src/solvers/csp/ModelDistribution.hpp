#ifndef TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP
#define TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP

#include <string.h>
#include <map>
#include <vector>
#include <gecode/set.hh>
#include <gecode/search.hh>

#include <templ/Mission.hpp>

namespace templ {
namespace solvers {
namespace csp {

struct LocationTimeService
{
    uint32_t location;
    uint32_t time;
    uint32_t service;
};

typedef std::map<LocationTimeService, std::vector<uint32_t> > Solution;

class ModelDistribution : public Gecode::Space
{

private:
    /**
     * Find a solution for the model distribution
     */
    ModelDistribution* solve();

    Solution getSolution() const;

    // TimeInterval -- Location (StateVariable) : associated robot models
    // pair(time_interval, location) -- map_to --> service requirements
    // pair(time_interval, location) -- map_to --> set of set of models
    //
    // optional:
    //  - parameterize on resource usage/distribution

protected:

public:
    ModelDistribution(const templ::Mission& mission);

    /**
     * Search support
     * This copy constructor is required for the search engine
     * and it has to provide a deep copy
     */
    ModelDistribution(bool share, ModelDistribution& s);

    /**
     * Creat a copy of this space
     * This method is called by the search engine
     */
    virtual Gecode::Space* copy(bool share);

    static std::vector<Solution> solve(const templ::Mission& mission);

    std::string toString() const;
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP
