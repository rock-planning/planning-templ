#include "FluentTimeResource.hpp"
#include <templ/solvers/csp/ModelDistribution.hpp>
#include <organization_model/Algebra.hpp>

#include <base-logging/Logging.hpp>

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

std::string FluentTimeResource::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "FluentTimeResource: " << std::endl;
    ss << hspace << "    resources: #";
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
    ss << hspace << "    time: #" << time << std::endl;
    ss << getInterval().toString(indent + 8) << std::endl;
    ss << hspace << "    fluent: #" << fluent << std::endl;
    ss << hspace << "        " << mission->getLocations()[fluent]->toString() << std::endl;
    ss << hspace << "    max cardinalities: " << std::endl
        << maxCardinalities.toString(indent + 8) << std::endl;
    ss << hspace << "    min cardinalities: " << std::endl
        << minCardinalities.toString(indent + 8) << std::endl;
    ss << hspace << "    model pool set: " << std::endl;
    organization_model::ModelPool::Set domain = getDomain();
    organization_model::ModelPool::Set::const_iterator dit = domain.begin();
    for(;dit != domain.end(); ++dit)
    {
        ss << dit->toString(indent + 8) << std::endl;
    }
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

Symbol::Ptr FluentTimeResource::getFluent() const
{
    return mission->getLocations()[fluent];
}

// TODO: this will require quantification as well
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
        }
    }
    return functionalities;
}

void FluentTimeResource::addFunctionalityConstraints(const organization_model::FunctionalityRequirement& constraint)
{
    organization_model::FunctionalityRequirement::Map::iterator it = functionalitiesConstraints.find(constraint.getFunctionality());
    if(it != functionalitiesConstraints.end())
    {
        it->second.addPropertyConstraints( constraint.getPropertyConstraints() );
    } else {
        functionalitiesConstraints[constraint.getFunctionality()] = constraint;
    }
}

void FluentTimeResource::compact(std::vector<FluentTimeResource>& requirements, const organization_model::OrganizationModelAsk& organizationModelAsk)
{
    std::vector<FluentTimeResource>::iterator it = requirements.begin();
    for(; it != requirements.end(); ++it)
    {
        FluentTimeResource& fts = *it;

        std::vector<FluentTimeResource>::iterator compareIt = it + 1;
        for(; compareIt != requirements.end();)
        {
            FluentTimeResource& otherFts = *compareIt;

            if(fts.time == otherFts.time && fts.fluent == otherFts.fluent)
            {
                LOG_DEBUG_S << "Compacting: " << std::endl
                    << fts.toString() << std::endl
                    << otherFts.toString() << std::endl;

                // Compacting the resource list
                fts.resources.insert(otherFts.resources.begin(), otherFts.resources.end());

                // Use the functional saturation bound on all functionalities
                // after compacting the resource list
                std::set<organization_model::Functionality> functionalities = fts.getFunctionalities();
                fts.maxCardinalities = organizationModelAsk.getFunctionalSaturationBound(functionalities);

                // MaxMin --> min cardinalities are a lower bound specified explicitely
                LOG_DEBUG_S << "Update Requirements: min: " << fts.minCardinalities.toString();
                LOG_DEBUG_S << "Update Requirements: otherMin: " << otherFts.minCardinalities.toString();

                fts.minCardinalities = organization_model::Algebra::max(fts.minCardinalities, otherFts.minCardinalities);
                LOG_DEBUG_S << "Result min: " << fts.minCardinalities.toString();

                // Resource constraints might enforce a minimum cardinality that is higher than the functional saturation bound
                // thus update the max cardinalities
                fts.maxCardinalities = organization_model::Algebra::max(fts.minCardinalities, fts.maxCardinalities);

                LOG_DEBUG_S << "Update requirement: " << fts.toString();

                requirements.erase(compareIt);
            } else {
                ++compareIt;
            }
        }
    }
    LOG_DEBUG_S << "END compact requirements";
}

organization_model::ModelPool::Set FluentTimeResource::getDomain() const
{
    assert(mission);
    using namespace organization_model;

    // The domain definition accounts for service requirements as well as explicitly stated
    // resource model requirements
    //
    // It constructs a model combination set, i.e., extensional constraints from
    // where solutions can be picked
    //
    // Collect functionality requirements
    std::set<organization_model::Functionality> functionalities = getFunctionalities();

    // When retrieving combinations for requested functionality, then this is not the
    // complete set since this might conflict with the minCardinalities
    // constraint -- thus we need to first derive the functionalBound and then
    // apply the minCardinalities by expanding the set of model if required
    organization_model::ModelPool::Set combinations = mission->getOrganizationModelAsk().getResourceSupport(functionalities, functionalitiesConstraints);

    LOG_INFO_S << "Resources: " << organization_model::ModelPool::toString(combinations);
    LOG_INFO_S << "    max cardinalities: " << maxCardinalities.toString(8);

    // Enforce the upper bound (defined e.g. by the missions general resource
    // constraints TBC)
    combinations = ModelPool::applyUpperBound(combinations, maxCardinalities);
    LOG_INFO_S << "Bounded resources (upper bound): " << ModelPool::toString(combinations);

    // The minimum resource requirements are accounted here by enforcing the
    // lower bound -- the given combinations are guaranteed to support the
    // services due upperBound which is derived from the functionalSaturationBound
    LOG_INFO_S << "    min cardinalities: " << minCardinalities.toString(8);
    combinations = ModelPool::expandToLowerBound(combinations, minCardinalities);
    LOG_INFO_S << "Expanded resource (lower bound enforced): " << ModelPool::toString(combinations);

    return combinations;
}


size_t FluentTimeResource::getIndex(const List& list, const FluentTimeResource& fluent)
{
    size_t index = 0;
    for(const FluentTimeResource& ftr : list)
    {
        if(ftr.time == fluent.time && ftr.fluent == fluent.fluent)
        {
            std::set<uint32_t> intersect;
            std::set_intersection(ftr.resources.begin(), ftr.resources.end(),
                    fluent.resources.begin(), fluent.resources.end(),
                    std::inserter(intersect, intersect.begin()) );

            if(intersect == fluent.resources)
            {
                return index;
            } else {
                std::cout << "no match, with intersect" << std::endl;
                for(uint32_t a : intersect)
                {
                    std::cout << a << " " << std::endl;
                }
            }
        }

        ++index;
    }
    }

    throw std::runtime_error("templ::solvers::csp::FluentTimeResource::getIndex: could not find fluent index for '" + fluent.toString() + "'");
}

void FluentTimeResource::incrementResourceRequirement(const owlapi::model::IRI& model, size_t number)
{
    owlapi::model::IRIList mappedResources = mission->getRequestedResources();
    for(size_t idx = 0; idx != mappedResources.size(); ++idx)
    {
        if(mappedResources[idx] == model)
        {
            resources.insert(idx);
            minCardinalities[model] += number;
            return;
        }
    }
    throw std::invalid_argument("FluentTimeResource::incrementResourceRequirement: failed to increment model '" + model.toString() + "'");
}

void FluentTimeResource::updateMaxCardinalities()
{
    using namespace organization_model;
    std::set<Functionality> functionalities = getFunctionalities();

    ModelPool saturation = mission->getOrganizationModelAsk().getFunctionalSaturationBound(functionalities, functionalitiesConstraints);

    ModelPool maxSaturationCardinalities = Algebra::min(mission->getAvailableResources(), saturation);
    // Resource constraints might enforce a minimum cardinality that is higher than the functional saturation bound
    // thus update the max cardinalities
    maxCardinalities = organization_model::Algebra::max(minCardinalities, maxSaturationCardinalities);
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
