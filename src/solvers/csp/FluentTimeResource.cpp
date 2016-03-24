#include "FluentTimeResource.hpp"
#include <templ/solvers/csp/ModelDistribution.hpp>

#include <base/Logging.hpp>

namespace templ {
namespace solvers {
namespace csp {

FluentTimeResource::FluentTimeResource()
{}

FluentTimeResource::FluentTimeResource(const Mission::Ptr& mission,
        uint32_t resource,
        uint32_t time,
        uint32_t fluent,
        const organization_model::ModelPool& availableModels)
    : mission(mission)
    , time(time)
    , fluent(fluent)
    , maxCardinalities(availableModels)
{
    assert(mission);
    resources.insert(resource);
}

bool FluentTimeResource::operator==(const FluentTimeResource& other) const
{
    return resources == other.resources && time == other.time && fluent == other.fluent;
}

bool FluentTimeResource::operator<(const FluentTimeResource& other) const
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

std::string FluentTimeResource::toString() const
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

std::vector< std::vector<FluentTimeResource> > FluentTimeResource::getConcurrent(const std::vector<FluentTimeResource>& requirements, const std::vector<solvers::temporal::Interval>& intervals)
{
    // map timeslot to fluenttime service
    std::map<uint32_t, std::vector<FluentTimeResource> > timeIndexedRequirements;
    {
        std::vector<FluentTimeResource>::const_iterator rit = requirements.begin();
        for(; rit != requirements.end(); ++rit)
        {
            const FluentTimeResource& fts = *rit;
            // map the time index
            timeIndexedRequirements[ rit->time ].push_back(fts);
        }
    }

    typedef std::vector<uint32_t> IndexCombination;
    typedef std::set< IndexCombination > IndexCombinationSet;
    IndexCombinationSet overlappingIntervals = solvers::temporal::Interval::overlappingIntervals(intervals);

    LOG_INFO_S << "Number of overlapping interval combinations: " << overlappingIntervals.size()
        << " from " << intervals.size() << " intervals overall";


    // All fluents that are on the same time overlap by default
    std::vector< std::vector<FluentTimeResource> > concurrentFts;
    std::map<uint32_t, std::vector<FluentTimeResource> >::const_iterator fit = timeIndexedRequirements.begin();
    for(; fit != timeIndexedRequirements.end(); ++fit)
    {
        concurrentFts.push_back(fit->second);
    }

    // All fluents that are in overlappping intervals overlap
    IndexCombinationSet::const_iterator cit = overlappingIntervals.begin();
    for(; cit != overlappingIntervals.end(); ++cit)
    {
        std::vector<FluentTimeResource> concurrent;

        const IndexCombination& indexCombination = *cit;
        IndexCombination::const_iterator iit = indexCombination.begin();
        for(; iit != indexCombination.end(); ++iit)
        {
            uint32_t timeIndex = *iit;
            const std::vector<FluentTimeResource>& fts = timeIndexedRequirements[ timeIndex ];

            concurrent.insert(concurrent.end(), fts.begin(), fts.end());
        }
        concurrentFts.push_back(concurrent);
    }

    return concurrentFts;
}

solvers::temporal::Interval FluentTimeResource::getInterval() const
{
    return mission->getTimeIntervals().at(time);
}

std::set<organization_model::Functionality> FluentTimeResource::getFunctionalities() const
{
    owlapi::model::OWLOntologyAsk ontologyAsk(mission->getOrganizationModel()->ontology());
    owlapi::model::IRIList mappedResources = mission->getRequestedResources();

    std::set<organization_model::Functionality> functionalities;

    std::set<uint32_t>::const_iterator cit = resources.begin();
    for(; cit != resources.end(); ++cit)
    {
        const owlapi::model::IRI& resourceModel = mappedResources[*cit];

        using namespace organization_model;
        if( ontologyAsk.isSubClassOf(resourceModel, organization_model::vocabulary::OM::Functionality()))
        {
            organization_model::Functionality functionality(resourceModel);
            functionalities.insert(functionality);
            LOG_INFO_S << "Add functionality requirement: " << functionality.toString();
        }
    }
    return functionalities;
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
