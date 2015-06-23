#ifndef TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP
#define TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP

#include <string.h>
#include <map>
#include <vector>
#include <gecode/set.hh>
#include <gecode/search.hh>

#include <templ/Mission.hpp>
#include <organization_model/OrganizationModelAsk.hpp>

namespace templ {
namespace solvers {
namespace csp {

struct FluentTimeService
{
    uint32_t service;
    uint32_t time;
    uint32_t fluent;

    FluentTimeService(uint32_t service, uint32_t time, uint32_t fluent)
        : service(service)
        , time(time)
        , fluent(fluent)
    {}
};

typedef std::map<FluentTimeService, std::vector<uint32_t> > Solution;

class ModelDistribution : public Gecode::Space
{
    owlapi::model::IRIList mServices;
    std::vector<solvers::temporal::Interval> mIntervals;
    std::vector<ObjectVariable::Ptr> mVariables;

    /// need to be translated into a set var
    std::vector<FluentTimeService> mRequirements;

    owlapi::model::IRIList mAvailableModels;

    organization_model::OrganizationModelAsk mAsk;
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
    organization_model::ModelCombinationSet getDomain(const FluentTimeService& requirement) const;

    std::set< std::vector<uint32_t> > toCSP(const organization_model::ModelCombinationSet& set) const;
    std::vector<uint32_t> toCSP(const organization_model::ModelCombination& combination) const;
    uint32_t systemModelToCSP(const owlapi::model::IRI& model) const;

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
