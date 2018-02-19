#include "FluentTimeResource.hpp"
#include <organization_model/Algebra.hpp>
#include <base-logging/Logging.hpp>
#include "../utils/Index.hpp"

namespace templ {
namespace solvers {

FluentTimeResource::FluentTimeResource()
{}

FluentTimeResource::FluentTimeResource(const Mission::Ptr& mpMission,
        size_t resource,
        size_t timeInterval,
        size_t fluent,
        const organization_model::ModelPool& availableModels)
    : mpMission(mpMission)
    , mTimeInterval(timeInterval)
    , mFluent(fluent)
    , mMaxCardinalities(availableModels)
{
    assert(mpMission);
    mResources.insert(resource);
}

bool FluentTimeResource::operator==(const FluentTimeResource& other) const
{
    return mResources == other.mResources && mTimeInterval == other.mTimeInterval && mFluent == other.mFluent;
}

bool FluentTimeResource::operator<(const FluentTimeResource& other) const
{
    if(mResources == other.mResources)
    {
        if(mTimeInterval == other.mTimeInterval)
        {
            return mFluent < other.mFluent;
        }
        return mTimeInterval < other.mTimeInterval;
    }
    return mResources < other.mResources;
}

std::string FluentTimeResource::toString(uint32_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "FluentTimeResource: " << std::endl;
    ss << hspace << "    resources: #";
    std::set<size_t>::const_iterator cit = mResources.begin();
    for(; cit != mResources.end(); )
    {
        ss << *cit;
        if(++cit != mResources.end())
        {
            ss << ",";
        }
    }
    ss << std::endl;
    ss << hspace << "    time: #" << mTimeInterval << std::endl;
    ss << getInterval().toString(indent + 8) << std::endl;
    ss << hspace << "    fluent: #" << mFluent << std::endl;
    ss << hspace << "        " << getLocation()->toString() << std::endl;
    ss << hspace << "    max cardinalities: " << std::endl
        << mMaxCardinalities.toString(indent + 8) << std::endl;
    ss << hspace << "    min cardinalities: " << std::endl
        << mMinCardinalities.toString(indent + 8) << std::endl;
    ss << hspace << "    sat cardinalities: " << std::endl
        << mSatisficingCardinalities.toString(indent + 8) << std::endl;
    ss << hspace << "    resulting domain: " << std::endl;
    organization_model::ModelPool::Set domain = getDomain();
    organization_model::ModelPool::Set::const_iterator dit = domain.begin();
    for(;dit != domain.end(); ++dit)
    {
        ss << dit->toString(indent + 8) << std::endl;
    }
    ss << hspace << "    constraints: " << std::endl;
    ss << organization_model::FunctionalityRequirement::toString(mFunctionalitiesConstraints, indent + 8) << std::endl;
    return ss.str();
}

std::string FluentTimeResource::toString(const List& list, uint32_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "FluentTimeResource List:" << std::endl;
    for(FluentTimeResource ftr : list)
    {
        ss << ftr.toString(indent + 4);
    }
    return ss.str();
}

solvers::temporal::Interval FluentTimeResource::getInterval() const
{
    return mpMission->getTimeIntervals().at(mTimeInterval);
}

void FluentTimeResource::setInterval(const solvers::temporal::Interval& interval)
{
    mTimeInterval = utils::Index::getIndex(mpMission->getTimeIntervals(), interval, "templ::solvers::FluentTimeResource::setInterval: failed to find interval: " + interval.toString());
}

void FluentTimeResource::setInterval(size_t time)
{
    if(time >= mpMission->getTimeIntervals().size())
    {
        throw std::invalid_argument("templ::solvers::FluentTimeResource::setInterval: time interval"
                    "exceeds index of available intervals");
    }
    mTimeInterval = time;
}

Symbol::Ptr FluentTimeResource::getFluent() const
{
    return mpMission->getLocations().at(mFluent);
}

void FluentTimeResource::setFluent(const Symbol::Ptr& symbol)
{
    symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(symbol);
    setLocation(location);
}

void FluentTimeResource::setFluent(size_t fluent)
{
    if(fluent >= mpMission->getLocations().size())
    {
        throw std::invalid_argument("templ::solvers::FluentTimeResource::setFluent: fluent idx "
                "exceeds available fluents (here: locations)");
    }
    mFluent = fluent;
}

symbols::constants::Location::Ptr FluentTimeResource::getLocation() const
{
    return dynamic_pointer_cast<symbols::constants::Location>(getFluent());
}

void FluentTimeResource::setLocation(const symbols::constants::Location::Ptr& location)
{
    mFluent = utils::Index::getIndex(mpMission->getLocations(), location, "templ::solvers::FluentTimeResource::setLocation: " + location->toString());
}


void FluentTimeResource::setMinCardinalities(const owlapi::model::IRI& model, size_t cardinality)
{
    mMinCardinalities[model] = cardinality;
}

void FluentTimeResource::setMaxCardinalities(const owlapi::model::IRI& model, size_t cardinality)
{
    mMaxCardinalities[model] = cardinality;
}

void FluentTimeResource::setSatisficingCardinalities(const owlapi::model::IRI& model, size_t cardinality)
{
    mSatisficingCardinalities[model] = cardinality;
}

std::vector< FluentTimeResource::List > FluentTimeResource::getConcurrent(const List& requirements,
        const std::vector<solvers::temporal::Interval>& intervals)
{
    // map timeslot to fluenttime service
    std::map<size_t, List > timeIndexedRequirements;
    {
        for(const FluentTimeResource& fts : requirements)
        {
            // map the time index
            timeIndexedRequirements[ fts.mTimeInterval ].push_back(fts);
        }
    }

    // All fluents that are on the same time overlap by default
    std::vector< List > concurrentFts;
    for(const std::pair<size_t, List>& mapEntry : timeIndexedRequirements)
    {
        concurrentFts.push_back(mapEntry.second);
    }

    // Identify overlapping intervals
    typedef std::vector<uint32_t> IndexCombination;
    typedef std::set< IndexCombination > IndexCombinationSet;
    IndexCombinationSet overlappingIntervals = solvers::temporal::Interval::overlappingIntervals(intervals);

    // All fluents that are in overlapping intervals also overlap
    for(const IndexCombination& indexCombination : overlappingIntervals)
    {
        std::vector<FluentTimeResource> concurrent;

        for(size_t timeIndex : indexCombination)
        {
            const std::vector<FluentTimeResource>& fts = timeIndexedRequirements[ timeIndex ];
            concurrent.insert(concurrent.end(), fts.begin(), fts.end());
        }
        concurrentFts.push_back(concurrent);
    }

    return concurrentFts;
}
// TODO: this will require quantification as well
std::set<organization_model::Functionality> FluentTimeResource::getFunctionalities() const
{
    owlapi::model::OWLOntologyAsk ontologyAsk(mpMission->getOrganizationModel()->ontology());
    owlapi::model::IRIList mappedResources = mpMission->getRequestedResources();

    std::set<organization_model::Functionality> functionalities;

    for(size_t resourceIdx : mResources)
    {
        const owlapi::model::IRI& resourceModel = mappedResources[resourceIdx];

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
    organization_model::FunctionalityRequirement::Map::iterator it = mFunctionalitiesConstraints.find(constraint.getFunctionality());
    if(it != mFunctionalitiesConstraints.end())
    {
        it->second.addPropertyConstraints( constraint.getPropertyConstraints() );
    } else {
        mFunctionalitiesConstraints[constraint.getFunctionality()] = constraint;
    }
}

void FluentTimeResource::compact(std::vector<FluentTimeResource>& requirements)
{
    LOG_DEBUG_S << "BEGIN compact requirements";
    std::vector<FluentTimeResource>::iterator it = requirements.begin();
    for(; it != requirements.end(); ++it)
    {
        FluentTimeResource& fts = *it;

        std::vector<FluentTimeResource>::iterator compareIt = it + 1;
        for(; compareIt != requirements.end();)
        {
            FluentTimeResource& otherFts = *compareIt;

            if(fts.mTimeInterval == otherFts.mTimeInterval && fts.mFluent == otherFts.mFluent)
            {
                LOG_DEBUG_S << "Compacting: " << std::endl
                    << fts.toString() << std::endl
                    << otherFts.toString() << std::endl;

                // Compacting the resource list
                fts.mResources.insert(otherFts.mResources.begin(), otherFts.mResources.end());
                fts.mMinCardinalities = organization_model::Algebra::max(fts.mMinCardinalities, otherFts.mMinCardinalities);
                fts.mMaxCardinalities = organization_model::Algebra::min(fts.mMaxCardinalities, otherFts.mMaxCardinalities);
                fts.mSatisficingCardinalities = organization_model::Algebra::max(fts.mSatisficingCardinalities, otherFts.mSatisficingCardinalities);

                LOG_DEBUG_S << "Result:" << fts.toString(8);
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
    assert(mpMission);
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
    //
    // The functional saturation bound has already been applied to the
    // organization model at initialization
    organization_model::ModelPool::Set combinations = mpMission->getOrganizationModelAsk().getResourceSupport(functionalities, mFunctionalitiesConstraints);

    LOG_INFO_S << "Identify resource support for the following functionalities: " << organization_model::Functionality::toString(functionalities) << std::endl
            << "    with constraints: " << std::endl << organization_model::FunctionalityRequirement::toString(mFunctionalitiesConstraints, 8) << std::endl
            << "    resources: " << organization_model::ModelPool::toString(combinations);

    LOG_INFO_S << "    max cardinalities: " << mMaxCardinalities.toString(8);

    // Enforce the upper bound (defined e.g. by the mpMissions general resource
    // constraints TBC)
    combinations = ModelPool::applyUpperBound(combinations, mMaxCardinalities);
    LOG_INFO_S << "Bounded resources (upper bound): " << ModelPool::toString(combinations);

    // The minimum resource requirements are accounted here by enforcing the
    // lower bound -- the given combinations are guaranteed to support the
    // services due upperBound which is derived from the functionalSaturationBound
    LOG_INFO_S << "    min cardinalities: " << mMinCardinalities.toString(8);
    combinations = ModelPool::expandToLowerBound(combinations, mMinCardinalities);
    LOG_INFO_S << "Expanded resource (lower bound enforced): " << ModelPool::toString(combinations);

    return combinations;
}


size_t FluentTimeResource::getIndex(const List& list, const FluentTimeResource& fluent)
{
    size_t index = 0;
    for(const FluentTimeResource& ftr : list)
    {
        if(ftr.mTimeInterval == fluent.mTimeInterval && ftr.mFluent == fluent.mFluent)
        {
            std::set<size_t> intersect;
            std::set_intersection(ftr.mResources.begin(), ftr.mResources.end(),
                    fluent.mResources.begin(), fluent.mResources.end(),
                    std::inserter(intersect, intersect.begin()) );

            if(intersect == fluent.mResources)
            {
                return index;
            }
        }

        ++index;
    }

    throw std::runtime_error("templ::solvers::csp::FluentTimeResource::getIndex: could not find fluent index for '" + fluent.toString() + "' in list of existing:\n'"
            + FluentTimeResource::toString(list,4) +"'");
}

void FluentTimeResource::incrementResourceRequirement(const owlapi::model::IRI& model, size_t number)
{
    owlapi::model::IRIList mappedResources = mpMission->getRequestedResources();
    for(size_t idx = 0; idx != mappedResources.size(); ++idx)
    {
        if(mappedResources[idx] == model)
        {
            mResources.insert(idx);
            mMinCardinalities[model] += number;
            return;
        }
    }
    throw std::invalid_argument("FluentTimeResource::incrementResourceRequirement: failed to increment model '" + model.toString() + "'");
}

void FluentTimeResource::updateSatisficingCardinalities()
{
    using namespace organization_model;
    std::set<Functionality> functionalities = getFunctionalities();

    ModelPool saturation = mpMission->getOrganizationModelAsk().getFunctionalSaturationBound(functionalities, mFunctionalitiesConstraints);
    ModelPool satisficingCardinalities = Algebra::min(mpMission->getAvailableResources(), saturation);

    // Resource constraints might enforce a minimum cardinality that is higher than the functional saturation bound
    // thus update the max cardinalities
    satisficingCardinalities = organization_model::Algebra::min(mMaxCardinalities, mSatisficingCardinalities);
    satisficingCardinalities = organization_model::Algebra::max(mMinCardinalities, mSatisficingCardinalities);
}

} // end namespace solvers
} // end namespace templ
