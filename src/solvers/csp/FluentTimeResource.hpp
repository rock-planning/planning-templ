#ifndef TEMPL_SOLVERS_CSP_FLUENT_TIME_RESOURCE_HPP
#define TEMPL_SOLVERS_CSP_FLUENT_TIME_RESOURCE_HPP

#include <set>
#include <vector>
#include <cstdint>
#include <organization_model/ModelPool.hpp>
#include <templ/solvers/temporal/Interval.hpp>

namespace templ {
namespace solvers {
namespace csp {

    class ModelDistribution;

struct FluentTimeResource
{
    std::set<uint32_t> resources;
    uint32_t time;
    uint32_t fluent;

    //ObjectVariable::Ptr objectVariable;

    // Set the min cardinality
    // of the available models
    organization_model::ModelPool minCardinalities;
    organization_model::ModelPool maxCardinalities;

    typedef std::vector<FluentTimeResource> List;

    FluentTimeResource(uint32_t resource, uint32_t time, uint32_t fluent,
            const organization_model::ModelPool& availableModels = organization_model::ModelPool());

    bool operator==(const FluentTimeResource& other) const;
    bool operator<(const FluentTimeResource& other) const;
    std::string toString() const;

    /**
     * Get the overlapping/concurrent FluentTimeResources
     * from indexed list of intervals
     * \param requirements Referencing intervals using index
     * \param intervals Intervallist that is reference by requirements
     */
    static std::vector< std::vector<FluentTimeResource> > getConcurrent(const std::vector<FluentTimeResource>& requirements,
            const std::vector<solvers::temporal::Interval>& intervals);

    solvers::temporal::Interval getInterval(const ModelDistribution* modelDistribution) const;
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_FLUENT_TIME_RESOURCE_HPP
