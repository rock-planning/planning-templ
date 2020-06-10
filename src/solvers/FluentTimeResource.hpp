#ifndef TEMPL_SOLVERS_FLUENT_TIME_RESOURCE_HPP
#define TEMPL_SOLVERS_FLUENT_TIME_RESOURCE_HPP

#include <set>
#include <vector>
#include <cstdint>
#include <owlapi/model/OWLOntologyAsk.hpp>
#include <moreorg/OrganizationModelAsk.hpp>
#include <moreorg/vocabularies/OM.hpp>
#include <moreorg/Resource.hpp>
#include "temporal/Interval.hpp"
#include "../symbols/constants/Location.hpp"

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
    FluentTimeResource(
            const moreorg::OrganizationModelAsk& ask,
            const owlapi::model::IRI& resource,
            symbols::constants::Location::Ptr& location,
            temporal::Interval timeInterval,
            const moreorg::ModelPool& availableModels = moreorg::ModelPool());

    bool operator==(const FluentTimeResource& other) const;
    bool operator<(const FluentTimeResource& other) const;

    std::string toString(uint32_t indent = 0) const;
    static std::string toString(const List& list, uint32_t indent = 0);

    void setOrganizationModelAsk(const moreorg::OrganizationModelAsk& ask) { mOrganizationModelAsk = ask; }
    const moreorg::OrganizationModelAsk& getOrganizationModelAsk() const { return mOrganizationModelAsk; }

    /**
     * Compute only the spatio-temporal qualification for identification of the
     * set of intervals
     */
    static std::string toQualificationString(const List& list, uint32_t indent = 0);

    template<class InputIterator>
    static std::vector<std::string> toQualificationStringList(InputIterator begin, InputIterator end)
    {
        std::vector<std::string> list;
        InputIterator it = begin;
        for(; it != end; ++it)
        {
            list.push_back(it->getQualificationString());
        }
        return list;
    }


    template<class InputIterator>
    static std::string toQualificationString(InputIterator begin,
            InputIterator end, uint32_t indent = 0, bool compact = false)
    {
        std::string hspace(indent,' ');
        std::stringstream ss;
        InputIterator it = begin;
        if(compact)
        {
            ss << hspace;
        }
        for(; it != end; ++it)
        {
            if(compact)
            {
                ss << it->getQualificationString() << ",";
            } else {
                ss << hspace << it->getQualificationString() << std::endl;
            }
        }
        return ss.str();
    }

    /**
     * Retrieve the interval associated with FluentTimeResource instance
     * \return interval this object refers to
     */
    const solvers::temporal::Interval& getInterval() const { return mTimeInterval; }

    /**
     * Set the interval index using the interval
     * \param interval
     * \throws std::invalid_argument if interval cannot be found in the mission
     */
    void setInterval(const solvers::temporal::Interval& interval) { mTimeInterval = interval; }

    /**
     * Get the associated fluent (here: location)
     * \return fluent symbol (currently only symbols::constants::Location::Ptr)
     */
    Symbol::Ptr getFluent() const;

    /**
     * Retrieve the fluent idx
     */
    size_t getFluentIdx() const { return mFluentIdx; }

    /**
     * Set fluent via symbol pointer
     * \param symbol
     * \throws std::invalid_argument when symbol cannot be found in the mission
     */
    void setFluent(const Symbol::Ptr& symbol);

    /**
     * Get location (fluent)
     */
    symbols::constants::Location::Ptr getLocation() const;

    /**
     * Set location index using the constant
     * \throw std::invalid_argument if the constant cannot be found in the
     * mission
     */
    void setLocation(const symbols::constants::Location::Ptr& location) {
        mpLocation = location; }

    /**
     * Get the minimum cardinalities for a number of resources
     */
    const moreorg::ModelPool& getMinCardinalities() const { return mMinCardinalities; }

    /**
     * Set the minimum cardinalities for resources
     */
    void setMinCardinalities(const moreorg::ModelPool& m) { mMinCardinalities = m; }

    /**
     * Set the minimum cardinality for a particular resource
     */
    void setMinCardinalities(const owlapi::model::IRI& model, size_t cardinality);

    /**
     * Get the maximum cardinalities for the required resources
     */
    const moreorg::ModelPool& getMaxCardinalities() const { return mMaxCardinalities; }

    /**
     * Set the maximum cardinalities for resources
     */
    void setMaxCardinalities(const moreorg::ModelPool& m) { mMaxCardinalities = m; }

    /**
     * Set the maximum cardinality for a particular resource
     */
    void setMaxCardinalities(const owlapi::model::IRI& model, size_t cardinality);

    /**
     * Sort requirements based on earlier end point
     * \param requirements List of requirements
     * \param tcp TimePointComparator which is used for temporal comparison
     */
    static void sortForEarlierEnd(List& requirements,
            temporal::point_algebra::TimePointComparator tpc);

    /**
     * Sort requirements based on earlier start point
     * \param requirements List of requirements
     * \param tcp TimePointComparator which is used for temporal comparison
     */
    static void sortForEarlierStart(List& requirements,
            temporal::point_algebra::TimePointComparator tpc);

    /**
     * Get the pairwise mutual exclusive set of requirements,
     * two requirements are mutually exclusive, when they refer to different
     * locations and their time interval overlaps
     * \param requirements List of requirements
     * \param tcp TimePointComparator which is used for temporal comparison
     */
    static std::vector<List> getMutualExclusive(const List& requirements,
            temporal::point_algebra::TimePointComparator tpc
            );

    /**
     * Get overlapping requirement,
     * requirements overlap, when they refer to the same location
     * and their time interval overlaps
     * \param requirements List of requirements
     * \param tcp TimePointComparator which is used for temporal comparison
     */
    static std::vector<Set> getOverlapping(const List& requirements,
            temporal::point_algebra::TimePointComparator tpc
            );

    /**
     * Create set of non overlapping requirements assuming requirements
     * which refer to the same location
     * \param requirements Existing requirements
     * \param sortedTimepoints Timepoint fully temporally ordered
     * \param tpc TimePointComparator which applies (for interval comparison)
     */
    static List createNonOverlappingRequirements(const List& requirements,
            temporal::point_algebra::TimePoint::PtrList sortedTimepoints,
            temporal::point_algebra::TimePointComparator tpc
            );

    /**
     * Test if functionalities are required
     * \return true if functionalities are required
     */
    bool hasFunctionalityConstraint() const;

    /**
     * Get the set of functionalities this FluentTimeResource requires
     * This is a subset of the overall required resources
     */
    moreorg::Resource::Set getRequiredResources() const;

    /**
     * Set the functionalities, i.e. overwrite all existing
     */
    void setRequiredResources(const moreorg::Resource::Set& resources) { mRequiredResources = resources; }

    /**
     * Add a functionality constraint
     * \param constraint Functionality constraint
     */
    void addRequiredResource(const moreorg::Resource& resource);

    /**
     * Create a compact representation for all requirement that
     * refer to the same fluent and time
     */
    static void compact(std::vector<FluentTimeResource>& requirements);

    /**
     * Merge other requirements into this one, while keeping
     * the time interval and fluent of this
     */
    void merge(const FluentTimeResource& otherFtr);

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
    moreorg::ModelPool::Set getDomain() const;

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
     */
    static bool areMutualExclusive(const FluentTimeResource& a , const FluentTimeResource& b,
            temporal::point_algebra::TimePointComparator tpc
            );

    static bool areOverlapping(const FluentTimeResource& a , const FluentTimeResource& b,
            temporal::point_algebra::TimePointComparator tpc
            );

    /**
     * Get qualification string as: (location,[fromTime,toTime])
     */
    std::string getQualificationString() const;

    void updateIndices(const symbols::constants::Location::PtrList& locations);

    static void updateIndices(List& requirements,
            const symbols::constants::Location::PtrList& locations);

private:
    /// Allow to map between indexes and symbols
    moreorg::OrganizationModelAsk mOrganizationModelAsk;

    /// Location idx for processing in CSP
    size_t mFluentIdx;

    /// involved resource types
    owlapi::model::IRISet mResources;
    /// the fluent, e.g. space/location
    symbols::constants::Location::Ptr mpLocation;
    /// the time interval index
    temporal::Interval mTimeInterval;

    // Set of required resources including additional constraints
    mutable moreorg::Resource::Set mRequiredResources;

    /// min cardinalities of the available models
    moreorg::ModelPool mMinCardinalities;
    /// max cardinalities of the available models
    moreorg::ModelPool mMaxCardinalities;
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_FLUENT_TIME_RESOURCE_HPP
