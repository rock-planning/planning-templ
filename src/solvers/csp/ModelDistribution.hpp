#ifndef TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP
#define TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP

#include <string>
#include <map>
#include <vector>
#include <gecode/set.hh>
#include <gecode/search.hh>

#include <organization_model/OrganizationModelAsk.hpp>

#include <templ/Mission.hpp>
#include <templ/solvers/csp/FluentTimeResource.hpp>

namespace templ {
namespace solvers {
namespace csp {

/**
 * \class ModelDistribution
 * \details ModelDistribution tries to associate actor models
 * according to the requirement of a given mission.
 *
 *
 */
class ModelDistribution : public Gecode::Space
{
public:
    typedef std::map<FluentTimeResource, organization_model::ModelPool > Solution;
    typedef std::vector<Solution> SolutionList;

private:
    /// The mission to plan for
    Mission mMission;

    /// The model pool -- in terms of available resources that can be used for
    /// planning
    organization_model::ModelPool mModelPool;
    /// The organization model
    organization_model::OrganizationModelAsk mAsk;

    /// All available services
    owlapi::model::IRIList mServices;
    /// All available resources
    owlapi::model::IRIList mResources;

    /// (Time)Intervals (as defined in the mission)
    std::vector<solvers::temporal::Interval> mIntervals;
    /// Constants: Locations (as defined in the mission)
    std::vector<symbols::constants::Location::Ptr> mLocations;

    /// Service Requirement that arise from the mission scenario
    std::vector<FluentTimeResource> mResourceRequirements;

    // map timeslot to fluenttime service
    std::map<uint32_t, std::vector<FluentTimeResource> > mTimeIndexedRequirements;

    // Array which will be accessed using Gecode::Matrix
    // The array contains information about the robot types required to
    // fullfill a requirement, e.g., r-0
    //
    // Additional resource constraints apply due to the upper resource
    // bound and time
    //  requirement | robot-type-0 | robot-type-1 | robot-type-2 | ...
    //  r-0         |      1       |      0       |      1       | ...
    //  r-1         |      0       |      2       |      2       | ...
    Gecode::IntVarArray mModelUsage;
    owlapi::model::IRIList mAvailableModels;

    // Per requirement: collect the extensional constraints
    std::map<uint32_t, Gecode::TupleSet> mExtensionalConstraints;

private:
    /**
     * Get the solution of this Gecode::Space instance
     */
    Solution getSolution() const;

    // TimeInterval -- Location (StateVariable) : associated robot models
    // pair(time_interval, location) -- map_to --> service requirements
    // pair(time_interval, location) -- map_to --> set of set of models
    //
    // optional:
    //  - parameterize on resource usage/distribution

    // Get the minimum requirements as set of ModelCombinations
    // \return ModelCombinations that fulfill the requirement
    organization_model::ModelCombinationSet getDomain(const FluentTimeResource& requirement) const;

    std::set< std::vector<uint32_t> > toCSP(const organization_model::ModelCombinationSet& set) const;
    std::vector<uint32_t> toCSP(const organization_model::ModelCombination& combination) const;
    uint32_t systemModelToCSP(const owlapi::model::IRI& model) const;

    /**
     * Convert the given list of combinations to a Gecode::TupleSet, i.e. create
     * extensional constraints out of the combination set
     * \return TupleSet
     */
    void appendToTupleSet(Gecode::TupleSet& tupleSet, const organization_model::ModelCombinationSet& combinations) const;

    size_t getFluentIndex(const FluentTimeResource& fluent) const;
    size_t getResourceModelIndex(const owlapi::model::IRI& model) const;
    const owlapi::model::IRI& getResourceModelFromIndex(size_t index) const;
    size_t getResourceModelMaxCardinality(size_t model) const;

    std::vector<FluentTimeResource> getResourceRequirements() const;

    size_t getMaxResourceCount(const organization_model::ModelPool& model) const;

    /**
     * Create a compact representation for all requirement that
     * refer to the same fluent and time
     */
    void compact(std::vector<FluentTimeResource>& requirements) const;
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

    static SolutionList solve(const templ::Mission& mission);

    std::string toString() const;
    void print(std::ostream& os) const { os << toString() << std::endl; }
};

std::ostream& operator<<(std::ostream& os, const ModelDistribution::Solution& solution);
std::ostream& operator<<(std::ostream& os, const ModelDistribution::SolutionList& solutions);

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP
