#ifndef TEMPL_SOLVERS_FLUENT_TIME_RESOURCE_HPP
#define TEMPL_SOLVERS_FLUENT_TIME_RESOURCE_HPP

#include <set>
#include <vector>
#include <cstdint>
#include <owlapi/model/OWLOntologyAsk.hpp>
#include <organization_model/ModelPool.hpp>
#include <organization_model/vocabularies/OM.hpp>
#include <organization_model/Resource.hpp>
#include <templ/solvers/temporal/Interval.hpp>
#include <templ/Mission.hpp>

namespace templ {
namespace solvers {

/**
 * \class FluentTimeResource
 * \details A FluentTimeResource represents a spatio-temporal requirement
 * defining the set of resources at a particular location over a given time
 * interval
 */
class FluentTimeResource
{

public:
    typedef std::vector<FluentTimeResource> List;
    typedef std::set<FluentTimeResource> Set;

    /**
     * Default constructor to allow usage in lists
     */
    FluentTimeResource();

    /**
     * Construct a FluentTimeResource
     * \param mission Mission from which this requirement originates, in order
     * to allow access to the organization model and available resources
     * \param resource Index of resource
     * \param time
     */
    FluentTimeResource(const Mission::Ptr& mission,
            size_t resource,
            size_t timeInterval,
            size_t fluent,
            const organization_model::ModelPool& availableModels = organization_model::ModelPool());

    bool operator==(const FluentTimeResource& other) const;
    bool operator<(const FluentTimeResource& other) const;

    std::string toString(uint32_t indent = 0) const;
    static std::string toString(const List& list, uint32_t indent = 0);

    /**
     * Set the mission that is associated with this FluentTimeResource instance
     */
    void setMission(const Mission::Ptr& mission) { mpMission = mission; }

    /**
     * Get the mission that is associated with this FluentTimeResource instance
     */
    Mission::Ptr getMission() const { return mpMission; }

    /**
     * Add a resource requirement using the idx (for CSP based variants)
     */
    void addResourceIdx(size_t idx);

    /**
     * Get resource requirement as set of indices
     */
    const std::set<size_t>& getResourceIndices() const { return mResources; }

    /**
     * Retrieve the interval associated with FluentTimeResource instance
     * \return interval this object refers to
     */
    solvers::temporal::Interval getInterval() const;

    /**
     * Retrieve the time interval index
     */
    size_t getTimeIntervalIdx() const { return mTimeInterval; }

    /**
     * Set the interval index using the interval
     * \param interval
     * \throws std::invalid_argument if interval cannot be found in the mission
     */
    void setInterval(const solvers::temporal::Interval& interval);

    /**
     * Set the interval index
     * \param time
     * \throw std::invalid_arugment if the interval index exceeds the number of
     * existing intervals
     */
    void setInterval(size_t time);

    /**
     * Get the associated fluent (here: location)
     * \return fluent symbol (currently only symbols::constants::Location::Ptr)
     */
    Symbol::Ptr getFluent() const;

    /**
     * Retrieve the fluent idx
     */
    size_t getFluentIdx() const { return mFluent; }

    /**
     * Set fluent via symbol pointer
     * \param symbol
     * \throws std::invalid_argument when symbol cannot be found in the mission
     */
    void setFluent(const Symbol::Ptr& symbol);

    /**
     * Set the fluent index
     * \param fluent Fluent index
     * \throw std::invalid_arugment if the index exceeds the number of
     * existing fluents
     */
    void setFluent(size_t fluent);

    /**
     * Get location (fluent)
     */
    symbols::constants::Location::Ptr getLocation() const;

    /**
     * Set location index using the constant
     * \throw std::invalid_argument if the constant cannot be found in the
     * mission
     */
    void setLocation(const symbols::constants::Location::Ptr& location);

    /**
     * Get the minimum cardinalities for a number of resources
     */
    const organization_model::ModelPool& getMinCardinalities() const { return mMinCardinalities; }

    /**
     * Set the minimum cardinalities for resources
     */
    void setMinCardinalities(const organization_model::ModelPool& m) { mMinCardinalities = m; }

    /**
     * Set the minimum cardinality for a particular resource
     */
    void setMinCardinalities(const owlapi::model::IRI& model, size_t cardinality);

    /**
     * Get the maximum cardinalities for the required resources
     */
    const organization_model::ModelPool& getMaxCardinalities() const { return mMaxCardinalities; }

    /**
     * Set the maximum cardinalities for resources
     */
    void setMaxCardinalities(const organization_model::ModelPool& m) { mMaxCardinalities = m; }

    /**
     * Set the maximum cardinality for a particular resource
     */
    void setMaxCardinalities(const owlapi::model::IRI& model, size_t cardinality);

    /**
     * Get the satisficing cardinalities for the required resources
     */
    const organization_model::ModelPool& getSatisficingCardinalities() const { return mSatisficingCardinalities; }

    /**
     * Set the satisficing cardinalities for the required resources
     */
    void setSatisficingCardinalities(const organization_model::ModelPool& m) { mSatisficingCardinalities = m; }

    /**
     * Set the satisficing cardinality for a particular resource
     */
    void setSatisficingCardinalities(const owlapi::model::IRI& model, size_t cardinality);

    /**
     * Get the overlapping/concurrent FluentTimeResources
     * from indexed list of intervals
     * \param requirements Referencing intervals using index
     * \param intervals Intervallist that is referenced by the requirements
     */
    static std::vector< List > getConcurrent(const List& requirements,
            const std::vector<solvers::temporal::Interval>& intervals);

    /**
     * Get the mutual exclusive set of requirements
     */
    static std::vector<List> getMutualExclusive(const List& requirements);

    /**
     * Get the set of functionalities this FluentTimeResource requires
     * This is a subset of the overall required resources
     */
    organization_model::Resource::Set getRequiredResources() const;

    /**
     * Set the functionalities, i.e. overwrite all existing
     */
    void setRequiredResources(const organization_model::Resource::Set& resources) { mRequiredResources = resources; }

    /**
     * Add a functionality constraint
     * \param constraint Functionality constraint
     */
    void addRequiredResource(const organization_model::Resource& resource);

    /**
     * Refresh the set of required resources
     */
    void updateRequiredResources() const;

    /**
     * Create a compact representation for all requirement that
     * refer to the same fluent and time
     */
    static void compact(std::vector<FluentTimeResource>& requirements);

    /**
     * Get the domain in terms of model pools that are allowed
     * TimeInterval -- Location (StateVariable) : associated robot models
     * pair(time_interval, location) -- map_to --> service requirements
     * pair(time_interval, location) -- map_to --> set of set of models
     *
     * optional:
     *  - parameterize on resource usage/distribution

     * \return ModelPools that fulfill the requirement
     */
    organization_model::ModelPool::Set getDomain() const;

    /**
     * Get the index of a fluent in a list of fluents
     * \param list List of FluentTimeResource
     * \param fluent
     */
    static size_t getIndex(const List& list, const FluentTimeResource& fluent);

    /**
     * Increment the min cardinality for a resource requirement for a given
     * increment, adds the resource model to the required resources if not
     * previously requested.
     * \param increment Value to add to the min cardinalities
     */
    void incrementResourceMinCardinality(const owlapi::model::IRI& model, size_t increment);

    /**
     * Update the set of satisficing cardinalities by computing the functional
     * saturation bound for the given functionalities and the existing
     * organization model ask (which accounts for the overall available
     * resources)
     *
     * Satisficying cardinalities has a lower bound at the minCardinalities and
     * upper bound at the maxCardinalities
     *
     */
    void updateSatisficingCardinalities();

private:
    /// Allow to map between indexes and symbols
    Mission::Ptr mpMission;

    /// involved resource types
    std::set<size_t> mResources;
    /// the time interval index
    size_t mTimeInterval;
    /// the fluent, e.g. space/location
    size_t mFluent;

    // Set of required resources including additional constraints
    mutable organization_model::Resource::Set mRequiredResources;

    /// min cardinalities of the available models
    organization_model::ModelPool mMinCardinalities;
    /// max cardinalities of the available models
    organization_model::ModelPool mMaxCardinalities;

    /// satisficing cardinalities (functional saturation) of the available
    /// models
    organization_model::ModelPool mSatisficingCardinalities;

    /// Flag to mark that an update is required
    /// see updateRequiredResources
    mutable bool mUpdateRequired;
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_FLUENT_TIME_RESOURCE_HPP
