#include "FluentTimeResource.hpp"
#include <organization_model/Algebra.hpp>
#include <base-logging/Logging.hpp>
#include <numeric/Combinatorics.hpp>
#include "../utils/Index.hpp"
#include <iostream>
#include <algorithm>

namespace templ {
namespace solvers {

FluentTimeResource::FluentTimeResource()
    : mUpdateRequired(false)
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
    , mUpdateRequired(true)
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
    ss << hspace << "    required resources: " << std::endl;
    ss << organization_model::Resource::toString(mRequiredResources, indent + 8) << std::endl;
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

void FluentTimeResource::addResourceIdx(size_t idx)
{
    mResources.insert(idx);
    mUpdateRequired = true;
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

void FluentTimeResource::sortForMutualExclusion(List& requirements,
        temporal::point_algebra::TimePointComparator tpc)
{
    std::sort(requirements.begin(), requirements.end(), [tpc](const FluentTimeResource& a,
                const FluentTimeResource& b)
            {
                if( tpc.lessThan(a.getInterval().getTo(),(b.getInterval().getTo())) )
                {
                    return true;
                } else if(tpc.equals(a.getInterval().getTo(), b.getInterval().getTo()))
                {
                    return tpc.lessThan(a.getInterval().getFrom(), b.getInterval().getFrom());
                }
                return false;
            });
}

std::vector<FluentTimeResource::List> FluentTimeResource::getMutualExclusive(const List& _requirements,
        temporal::point_algebra::TimePointComparator tpc
        )
{
    List requirements = _requirements;
    sortForMutualExclusion(requirements, tpc);

    // Todo: check keyword 'edge finding'

    // Take advantage of a specially ordered list:
    // based on (1) earlier end point means <= true,
    // earlier start time means <= true,
    // to as a result order all intervals overlapping with the smallest unit
    // will be concurrent
    // use sorted fluents, check overlap of the first interval with rest
    // and thus step through each interval and the potentially overlapping one
    // [a_s -------------------------------a_e]
    // [b_s --- b_e]
    //                      [c_s  --- c_e]
    //                                      [d_s ---- d_e]
    //
    //                 [e_s --- e_e]

    std::vector<List> mutualExclusive;
    std::vector< std::vector<std::string> > processedConcurrent;
    for(size_t a = 0; a < requirements.size();++a)
    {
        const FluentTimeResource& ftrA = requirements[a];

        std::vector<std::string> concurrentEncoded;;
        List concurrent;
        for(size_t b = a + 1; b < requirements.size(); ++b)
        {
            const FluentTimeResource& ftrB = requirements[b];
            if(FluentTimeResource::areMutualExclusive(ftrA, ftrB, tpc))
            {
                concurrent.push_back(ftrB);
                concurrentEncoded.push_back(ftrB.getQualificationString());
            } else {
                break;
            }
        }

        if(concurrent.empty())
        {
            mutualExclusive.push_back({ ftrA });
            continue;
        } else {
            bool alreadyProcessed = false;
            std::sort(concurrentEncoded.begin(), concurrentEncoded.end());
            for(std::vector<std::string>& processed : processedConcurrent)
            {
                if(std::includes(processed.begin(), processed.end(),
                        concurrentEncoded.begin(),
                        concurrentEncoded.end()) )
                {
                    // already processed so ignore this one
                    alreadyProcessed = true;
                }
            }
            if(alreadyProcessed)
            {
                continue;
            } else {
                processedConcurrent.push_back(concurrentEncoded);
            }
        }

        LOG_DEBUG_S << "Reference interval: " << std::endl
              << "   "
                  << ftrA.getQualificationString()
                  << std::endl
              << "    is concurrent with:" <<  std::endl
              << FluentTimeResource::toQualificationStringList(concurrent, 8);

        // Now concurrent, but not necessarily mutual exclusive intervals
        // are identified, e.g.
        // (l0,[t0,t1]) - (l1,[t0,t1]) - (l1,[t1,t2])
        // so check on full mutual exclusion on this subset
        std::vector<List> mutexGroups;
        mutexGroups.push_back( { concurrent.back() });
        concurrent.pop_back();
        while(!concurrent.empty())
        {
            FluentTimeResource ftrC = concurrent.back();
            concurrent.pop_back();

            std::vector<List> extraGroups;
            for(List& requirementList : mutexGroups)
            {
                List tmpMutualExclusive;
                bool newGroup = false;
                for(FluentTimeResource& ftr : requirementList)
                {
                    // create a new mutex group when this requirement
                    // is not mutual exclusive with all(!) requirement that
                    // are part of the current group
                    if(!FluentTimeResource::areMutualExclusive(ftr,ftrC, tpc))
                    {
                        tmpMutualExclusive.push_back(ftrC);
                        extraGroups.push_back(tmpMutualExclusive);
                        newGroup = true;
                        break;
                    }
                    tmpMutualExclusive.push_back(ftr);
                }

                if(!newGroup)
                {
                    requirementList.push_back(ftrC);
                }
            }
            for(List list : extraGroups)
            {
                mutexGroups.push_back(list);
            }
        }

        for(List c : mutexGroups)
        {
            c.push_back(ftrA);
            mutualExclusive.push_back(c);
        }
    }
    return mutualExclusive;
}


std::vector<FluentTimeResource::List> FluentTimeResource::getOverlapping(const List& _requirements,
        temporal::point_algebra::TimePointComparator tpc
        )
{
    List requirements = _requirements;
    sortForMutualExclusion(requirements, tpc);

    // Take advantage of a specially ordered list:
    // based on (1) earlier end point means <= true,
    // earlier start time means <= true,
    // to as a result order all intervals overlapping with the smallest unit
    // will be concurrent
    // use sorted fluents, check overlap of the first interval with rest
    // and thus step through each interval and the potentially overlapping one
    // [a_s -------------------------------a_e]
    // [b_s --- b_e]
    //                      [c_s  --- c_e]
    //                                      [d_s ---- d_e]
    //
    //                 [e_s --- e_e]

    std::vector<List> overlapping;
    std::vector< std::vector<std::string> > processedConcurrent;
    for(size_t a = 0; a < requirements.size();++a)
    {
        const FluentTimeResource& ftrA = requirements[a];

        std::vector<std::string> concurrentEncoded;;
        List concurrent;
        for(size_t b = a + 1; b < requirements.size(); ++b)
        {
            const FluentTimeResource& ftrB = requirements[b];
            if(FluentTimeResource::areOverlapping(ftrA, ftrB, tpc))
            {
                concurrent.push_back(ftrB);
            } else {
                // due to the sorting we can stop here, since no
                // other interval will overlap
                break;
            }
        }
    }
    return overlapping;
}

bool FluentTimeResource::hasFunctionalityConstraint() const
{
    for(const organization_model::Resource& resource : mRequiredResources)
    {
        if( mpMission->getOrganizationModelAsk().ontology().isSubClassOf(resource.getModel(),
                    organization_model::vocabulary::OM::Functionality()))
        {
            return true;
        }
    }
    return false;
}

organization_model::Resource::Set FluentTimeResource::getRequiredResources() const
{
    if(mUpdateRequired)
    {
        updateRequiredResources();
        mUpdateRequired = false;
    }

    return mRequiredResources;
}

// TODO: this will require quantification as well, but an be added as
// constraint, e.g. cardinality constraint to
void FluentTimeResource::updateRequiredResources() const
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
            organization_model::Resource::Set::const_iterator cit;
            cit = std::find_if(mRequiredResources.begin(), mRequiredResources.end(), [resourceModel](const organization_model::Resource& r)
                    {
                        return resourceModel == r.getModel();
                    });
            if(cit == mRequiredResources.end())
            {
                organization_model::Resource functionality(resourceModel);
                mRequiredResources.insert(functionality);
            }
        }
    }
}

void FluentTimeResource::addRequiredResource(const organization_model::Resource& resource)
{
    mRequiredResources.insert(resource);
    mRequiredResources = organization_model::Resource::merge(mRequiredResources);
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
    // Collect requiredResource, including functionality requirements
    organization_model::Resource::Set requiredResources = getRequiredResources();

    // When retrieving combinations for requested functionality, then this is not the
    // complete set since this might conflict with the minCardinalities
    // constraint -- thus we need to first derive the functionalBound and then
    // apply the minCardinalities by expanding the set of model if required
    //
    // The functional saturation bound has already been applied to the
    // organization model at initialization
    organization_model::ModelPool::Set combinations = mpMission->getOrganizationModelAsk().getResourceSupport(requiredResources);

    LOG_INFO_S << "Support for the following resource requests:" << std::endl
        << organization_model::Functionality::toString(requiredResources) << std::endl
        << "    agent combinations: " << organization_model::ModelPool::toString(combinations);

    LOG_INFO_S << "    max cardinalities: " << mMaxCardinalities.toString(8);

    // Enforce the upper bound (defined e.g. by the mpMissions general resource
    // constraints TBC)
    combinations = ModelPool::applyUpperBound(combinations, mMaxCardinalities);
    LOG_INFO_S << "Bounded resources (upper bound): " << ModelPool::toString(combinations);

    // The minimum resource requirements are accounted here by enforcing the
    // lower bound -- the given combinations are guaranteed to support the
    // services since the upperBound is derived from the functionalSaturationBound
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

void FluentTimeResource::incrementResourceMinCardinality(const owlapi::model::IRI& model, size_t number)
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
    Resource::Set requiredResources = getRequiredResources();

    ModelPool saturation = mpMission->getOrganizationModelAsk().getFunctionalSaturationBound(requiredResources);
    ModelPool satisficingCardinalities = Algebra::min(mpMission->getAvailableResources(), saturation);

    // Resource constraints might enforce a minimum cardinality that is higher than the functional saturation bound
    // thus update the max cardinalities
    satisficingCardinalities = organization_model::Algebra::min(mMaxCardinalities, mSatisficingCardinalities);
    satisficingCardinalities = organization_model::Algebra::max(mMinCardinalities, mSatisficingCardinalities);
}

bool FluentTimeResource::areMutualExclusive(const FluentTimeResource& ftrA,
            const FluentTimeResource& ftrB,
            temporal::point_algebra::TimePointComparator tpc)
{
    if(ftrA.getLocation() != ftrB.getLocation())
    {
        if(tpc.hasIntervalOverlap(ftrA.getInterval().getFrom(),
                    ftrA.getInterval().getTo(),
                    ftrB.getInterval().getFrom(),
                    ftrB.getInterval().getTo()))
        {
            return true;
        }
    }
    return false;
}

bool FluentTimeResource::areOverlapping(const FluentTimeResource& ftrA,
            const FluentTimeResource& ftrB,
            temporal::point_algebra::TimePointComparator tpc)
{
    if(ftrA.getLocation() == ftrB.getLocation())
    {
        if(tpc.hasIntervalOverlap(ftrA.getInterval().getFrom(),
                    ftrA.getInterval().getTo(),
                    ftrB.getInterval().getFrom(),
                    ftrB.getInterval().getTo()))
        {
            return true;
        }
    }
    return false;
}

std::string FluentTimeResource::getQualificationString() const
{
    std::stringstream ss;
    ss << "(" << getLocation()->getInstanceName() << ",";
    ss << "[" << getInterval().getFrom()->getLabel();
    ss << "," << getInterval().getTo()->getLabel() << "])";
    return ss.str();
}

std::string FluentTimeResource::toQualificationStringList(const List& list, uint32_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    for(const FluentTimeResource& ftr : list)
    {
        ss << hspace << ftr.getQualificationString() << std::endl;
    }
    return ss.str();
}

} // end namespace solvers
} // end namespace templ
