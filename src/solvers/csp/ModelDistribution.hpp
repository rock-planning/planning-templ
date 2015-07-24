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
    std::set<uint32_t> services;
    uint32_t time;
    uint32_t fluent;

    typedef std::vector<FluentTimeService> List;

    FluentTimeService(uint32_t service, uint32_t time, uint32_t fluent)
        : time(time)
        , fluent(fluent)
    {
        services.insert(service);
    }

    bool operator==(const FluentTimeService& other) const
    {
        return services == other.services && time == other.time && fluent == other.fluent;
    }

    bool operator<(const FluentTimeService& other) const
    {
        if(services == other.services)
        {
            if(time == other.time)
            {
                return fluent < other.fluent;
            }
            return time < other.time;
        }
        return services < other.services;
    }

    std::string toString() const
    {
        std::stringstream ss;
        ss << "FluentTimeService: " << std::endl;
        ss << "    services: #";
        std::set<uint32_t>::const_iterator cit = services.begin();
        for(; cit != services.end(); )
        {
            ss << *cit;
            if(++cit != services.end())
            {
                ss << ",";
            }
        }
        ss << std::endl;
        ss << "    time: #" << time << std::endl;
        ss << "    fluent: #" << fluent << std::endl;
        return ss.str();
    }

    /**
     * Get the overlapping/concurrent FluentTimeServices
     * from indexed list of intervals
     * \param requirements Referencing intervals using index
     * \param intervals Intervallist that is reference by requirements
     */
    static std::vector< std::vector<FluentTimeService> > getConcurrent(const std::vector<FluentTimeService>& requirements,
            const std::vector<solvers::temporal::Interval>& intervals);

};



class ModelDistribution : public Gecode::Space
{
public:
    typedef std::map<FluentTimeService, organization_model::ModelPool > Solution;
    typedef std::vector<Solution> SolutionList;

private:
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

    // Array which will be access using Gecode::Matrix
    // -- contains information about the robot types required to
    // fullfill r-0
    // Additional resource constraints apply due to the upper resource
    // bound and time
    //  requirement | robot-type-0 | robot-type-1 | robot-type-2 | ...
    //  r-0         |      1       |      0       |      1       | ...
    //  r-1         |      0       |      2       |      2       | ...
    Gecode::IntVarArray mModelUsage;
    owlapi::model::IRIList mAvailableModels;

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
    organization_model::ModelCombinationSet getDomain(const FluentTimeService& requirement) const;

    std::set< std::vector<uint32_t> > toCSP(const organization_model::ModelCombinationSet& set) const;
    std::vector<uint32_t> toCSP(const organization_model::ModelCombination& combination) const;
    uint32_t systemModelToCSP(const owlapi::model::IRI& model) const;

    /**
     * Convert the given list of combinations to a Gecode::TupleSet, i.e. create
     * extensional constraints out of the combination set
     * \return TupleSet
     */
    Gecode::TupleSet toTupleSet(const organization_model::ModelCombinationSet& combinations) const;

    size_t getFluentIndex(const FluentTimeService& fluent) const;
    size_t getResourceModelIndex(const owlapi::model::IRI& model) const;
    const owlapi::model::IRI& getResourceModelFromIndex(size_t index) const;
    size_t getResourceModelMaxCardinality(size_t model) const;

    std::vector<FluentTimeService> getRequirements() const;

    size_t getMaxResourceCount(const organization_model::ModelPool& model) const;

    void compact(std::vector<FluentTimeService>& requirements) const;
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
