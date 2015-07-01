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

    bool operator==(const FluentTimeService& other) const
    {
        return service == other.service && time == other.time && fluent == other.fluent;
    }

    bool operator<(const FluentTimeService& other) const
    {
        if(service == other.service)
        {
            if(time == other.time)
            {
                return fluent < other.fluent;
            }
            return time < other.time;
        }
        return service < other.service;
    }

    std::string toString() const
    {
        std::stringstream ss;
        ss << "FluentTimeService: " << std::endl;
        ss << "    service: #" << service << std::endl;
        ss << "    time: #" << time << std::endl;
        ss << "    fluent: #" << fluent << std::endl;
        return ss.str();
    }
};

typedef std::map<FluentTimeService, std::vector<uint32_t> > Solution;

class ModelDistribution : public Gecode::Space
{

    Mission mMission;
    organization_model::ModelPool mModelPool;
    organization_model::OrganizationModelAsk mAsk;

    owlapi::model::IRIList mServices;
    std::vector<solvers::temporal::Interval> mIntervals;
    std::vector<ObjectVariable::Ptr> mVariables;

    /// Requirement that arise from the mission scenario
    std::vector<FluentTimeService> mRequirements;

    // map timeslot to fluenttime service
    std::map<uint32_t, std::vector<FluentTimeService> > mTimeIndexedRequirements;

    /// Total domain for assignment of reconfigurable systems
    organization_model::ModelCombinationList mDomain;
    /// Domain for each requirement
    std::map<FluentTimeService, organization_model::ModelCombinationSet> mRequirementsDomain;

    Gecode::IntVarArray mModelUsage;
    owlapi::model::IRIList mAvailableModels;
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

    Gecode::TupleSet toTupleSet(const organization_model::ModelCombinationSet& combinations) const;

    size_t getFluentIndex(const FluentTimeService& fluent) const;
    size_t getResourceModelIndex(const owlapi::model::IRI& model) const;
    const owlapi::model::IRI& getResourceModelFromIndex(size_t index) const;
    size_t getResourceModelMaxCardinality(size_t model) const;

    std::vector<FluentTimeService> getRequirements() const;
    std::vector< std::vector<FluentTimeService> > getConcurrentRequirements() const;

    size_t getMaxResourceCount(const organization_model::ModelPool& model) const;

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
    void print(std::ostream& os) const { os << toString() << std::endl; }

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP
