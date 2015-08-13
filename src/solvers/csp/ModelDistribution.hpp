#ifndef TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP
#define TEMPL_SOLVERS_CSP_MODEL_DISTRIBUTION_HPP

#include <string.h>
#include <map>
#include <vector>
#include <gecode/set.hh>
#include <gecode/search.hh>
#include <iomanip>

#include <templ/Mission.hpp>
#include <organization_model/OrganizationModelAsk.hpp>

namespace templ {
namespace solvers {
namespace csp {

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
            const organization_model::ModelPool& availableModels = organization_model::ModelPool())
        : time(time)
        , fluent(fluent)
        , maxCardinalities(availableModels)
    {
        resources.insert(resource);
    }

    bool operator==(const FluentTimeResource& other) const
    {
        return resources == other.resources && time == other.time && fluent == other.fluent;
    }

    bool operator<(const FluentTimeResource& other) const
    {
        if(resources == other.resources)
        {
            if(time == other.time)
            {
                return fluent < other.fluent;
            }
            return time < other.time;
        }
        return resources < other.resources;
    }

    std::string toString() const
    {
        std::stringstream ss;
        ss << "FluentTimeResource: " << std::endl;
        ss << "    resources: #";
        std::set<uint32_t>::const_iterator cit = resources.begin();
        for(; cit != resources.end(); )
        {
            ss << *cit;
            if(++cit != resources.end())
            {
                ss << ",";
            }
        }
        ss << std::endl;
        ss << "    time: #" << time << std::endl;
        ss << "    fluent: #" << fluent << std::endl;
        ss << "    max cardinalities: " << maxCardinalities.toString() << std::endl;
        ss << "    min cardinalities: " << minCardinalities.toString() << std::endl;
        return ss.str();
    }

    /**
     * Get the overlapping/concurrent FluentTimeResources
     * from indexed list of intervals
     * \param requirements Referencing intervals using index
     * \param intervals Intervallist that is reference by requirements
     */
    static std::vector< std::vector<FluentTimeResource> > getConcurrent(const std::vector<FluentTimeResource>& requirements,
            const std::vector<solvers::temporal::Interval>& intervals);

};


class ConstraintMatrix
{
    struct MinMax
    {
        uint32_t min;
        uint32_t max;

        std::string toString() const
        {
            std::stringstream ss;
            ss << std::setw(4) << "(" << min << "," << max << ")";
            return ss.str();
        }
    };

    typedef uint32_t RowId, ColumnId;
    typedef MinMax Column;

    // Requirement mapped to the min,max setting
    std::map<RowId, std::map<ColumnId, MinMax> > mMatrix;
    owlapi::model::IRIList mAvailableModels;
public:

    ConstraintMatrix(const owlapi::model::IRIList& availableModels)
        : mAvailableModels(availableModels)
    {}

    void setMax(RowId row, ColumnId col, uint32_t max) {  mMatrix[row][col].max = max; }
    void setMin(RowId row, ColumnId col, uint32_t min) {  mMatrix[row][col].min = min; }

    std::string toString() const 
    {
        std::stringstream ss;
        ss << "Constraint matrix:" << std::endl;
        ss << "Available models: " << mAvailableModels << std::endl;
        for(auto row : mMatrix)
        {
            ss << "#" << std::setw(4) << row.first << " ";
            
            for(auto col : row.second)
            {
                ss << " " << col.second.toString();
            }
            ss << std::endl;
        }
        return ss.str();
    }

};

class ModelDistribution : public Gecode::Space
{
public:
    typedef std::map<FluentTimeResource, organization_model::ModelPool > Solution;
    typedef std::vector<Solution> SolutionList;

private:
    Mission mMission;
    organization_model::ModelPool mModelPool;
    organization_model::OrganizationModelAsk mAsk;

    owlapi::model::IRIList mServices;
    owlapi::model::IRIList mResources;
    std::vector<solvers::temporal::Interval> mIntervals;
    owlapi::model::IRIList mLocations;

    /// Service Requirement that arise from the mission scenario
    std::vector<FluentTimeResource> mResourceRequirements;

    // map timeslot to fluenttime service
    std::map<uint32_t, std::vector<FluentTimeResource> > mTimeIndexedRequirements;

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
