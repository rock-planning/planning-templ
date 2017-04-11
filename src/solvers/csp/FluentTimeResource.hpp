#ifndef TEMPL_SOLVERS_CSP_FLUENT_TIME_RESOURCE_HPP
#define TEMPL_SOLVERS_CSP_FLUENT_TIME_RESOURCE_HPP

#include <set>
#include <vector>
#include <cstdint>
#include <owlapi/model/OWLOntologyAsk.hpp>
#include <organization_model/ModelPool.hpp>
#include <organization_model/vocabularies/OM.hpp>
#include <organization_model/Functionality.hpp>
#include <templ/solvers/temporal/Interval.hpp>
#include <templ/Mission.hpp>

namespace templ {
namespace solvers {
namespace csp {

    class ModelDistribution;

struct FluentTimeResource
{
    Mission::Ptr mission;
    std::set<uint32_t> resources;
    uint32_t time;
    uint32_t fluent;

    //ObjectVariable::Ptr objectVariable;

    // Set the min cardinality
    // of the available models
    organization_model::ModelPool minCardinalities;
    organization_model::ModelPool maxCardinalities;

    typedef std::vector<FluentTimeResource> List;

    /**
     * Default (dummy) constructor
     */
    FluentTimeResource();

    FluentTimeResource(const Mission::Ptr& mission,
            uint32_t resource,
            uint32_t time,
            uint32_t fluent,
            const organization_model::ModelPool& availableModels = organization_model::ModelPool());

    bool operator==(const FluentTimeResource& other) const;
    bool operator<(const FluentTimeResource& other) const;
    std::string toString(uint32_t indent = 0) const;

    /**
     * Get the overlapping/concurrent FluentTimeResources
     * from indexed list of intervals
     * \param requirements Referencing intervals using index
     * \param intervals Intervallist that is reference by requirements
     */
    static std::vector< std::vector<FluentTimeResource> > getConcurrent(const std::vector<FluentTimeResource>& requirements,
            const std::vector<solvers::temporal::Interval>& intervals);

    solvers::temporal::Interval getInterval() const;

    Symbol::Ptr getFluent() const;

    /**
     * Get the set of functionalities this FluentTimeResource requires
     * \param ontology The ontology to map the resources
     * \param mappedResources A list of resources, where this FluentTimeResource
     * uses the index to refer to this resource -- allows to remap from index to
     * model
     */
    std::set<organization_model::Functionality> getFunctionalities() const;

    /**
     * Get the domain in terms of model pool that are allowed
     * TimeInterval -- Location (StateVariable) : associated robot models
     * pair(time_interval, location) -- map_to --> service requirements
     * pair(time_interval, location) -- map_to --> set of set of models
     *
     * optional:
     *  - parameterize on resource usage/distribution

     * Get the minimum requirements as set of ModelCombinations
     * \return ModelCombinations that fulfill the requirement
     */
    organization_model::ModelPoolSet getDomain() const;

    /**
     * Get the index of a fluent in a list of fluents
     */
    static size_t getIndex(const List& list, const FluentTimeResource& fluent);
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_FLUENT_TIME_RESOURCE_HPP
