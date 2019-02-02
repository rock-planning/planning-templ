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
    : mOrganizationModelAsk()
    , mFluentIdx(0)
{}

FluentTimeResource::FluentTimeResource(
        const organization_model::OrganizationModelAsk& ask,
        const owlapi::model::IRI& resourceModel,
        symbols::constants::Location::Ptr& location,
        temporal::Interval timeInterval,
        const organization_model::ModelPool& availableModels)
    : mOrganizationModelAsk(ask)
    , mFluentIdx(0)
    , mpLocation(location)
    , mTimeInterval(timeInterval)
    , mMaxCardinalities(availableModels)
{
    organization_model::Resource resource(resourceModel);
    addRequiredResource(resource);
}

bool FluentTimeResource::operator==(const FluentTimeResource& other) const
{
    return mResources == other.mResources && mTimeInterval ==
        other.mTimeInterval && mpLocation == mpLocation;
}

bool FluentTimeResource::operator<(const FluentTimeResource& other) const
{
    if(mResources == other.mResources)
    {
        if(mTimeInterval == other.mTimeInterval)
        {
            return mpLocation < other.mpLocation;
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
    ss << hspace << "    resources: ";
    owlapi::model::IRISet::const_iterator cit = mResources.begin();
    for(; cit != mResources.end(); )
    {
        ss << *cit;
        if(++cit != mResources.end())
        {
            ss << ",";
        }
    }
    ss << std::endl;
    ss << hspace << "    time:" << std::endl;
    ss << getInterval().toString(indent + 8) << std::endl;
    ss << hspace << "    fluent: #" << mFluentIdx << std::endl;
    ss << hspace << "        " << getLocation()->toString() << std::endl;
    ss << hspace << "    max cardinalities: " << std::endl
        << mMaxCardinalities.toString(indent + 8) << std::endl;
    ss << hspace << "    min cardinalities: " << std::endl
        << mMinCardinalities.toString(indent + 8) << std::endl;
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

Symbol::Ptr FluentTimeResource::getFluent() const
{
    return mpLocation;
}

void FluentTimeResource::setFluent(const Symbol::Ptr& symbol)
{
    symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(symbol);
    mpLocation = location;
}

symbols::constants::Location::Ptr FluentTimeResource::getLocation() const
{
    return mpLocation;
}

void FluentTimeResource::setMinCardinalities(const owlapi::model::IRI& model, size_t cardinality)
{
    mMinCardinalities[model] = cardinality;
}

void FluentTimeResource::setMaxCardinalities(const owlapi::model::IRI& model, size_t cardinality)
{
    mMaxCardinalities[model] = cardinality;
}

void FluentTimeResource::sortForEarlierEnd(List& requirements,
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

void FluentTimeResource::sortForEarlierStart(List& requirements,
        temporal::point_algebra::TimePointComparator tpc)
{
    std::sort(requirements.begin(), requirements.end(), [tpc](const FluentTimeResource& a,
                const FluentTimeResource& b)
            {
                if( tpc.lessThan(a.getInterval().getFrom(),(b.getInterval().getFrom())) )
                {
                    return true;
                } else if(tpc.equals(a.getInterval().getFrom(), b.getInterval().getFrom()))
                {
                    return tpc.lessThan(a.getInterval().getTo(), b.getInterval().getTo());
                }
                return false;
            });
}


std::vector<FluentTimeResource::List> FluentTimeResource::getMutualExclusive(const List& _requirements,
        temporal::point_algebra::TimePointComparator tpc
        )
{
    List requirements = _requirements;
    sortForEarlierStart(requirements, tpc);

    std::vector<List> mutualExclusive;
    for(size_t a = 0; a < requirements.size();++a)
    {
        const FluentTimeResource& ftrA = requirements[a];
        for(size_t b = a + 1; b < requirements.size(); ++b)
        {
            const FluentTimeResource& ftrB = requirements[b];
            if(FluentTimeResource::areMutualExclusive(ftrA, ftrB, tpc))
            {
                mutualExclusive.push_back({ ftrA, ftrB });
            } else {
                continue;
            }
        }
    }
    return mutualExclusive;
}


std::vector<FluentTimeResource::Set> FluentTimeResource::getOverlapping(const List& _requirements,
        temporal::point_algebra::TimePointComparator tpc
        )
{
    List requirements = _requirements;
    sortForEarlierEnd(requirements, tpc);

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

    using namespace temporal::point_algebra;
    std::vector<Set> overlapping;
    for(size_t a = 0; a < requirements.size();++a)
    {
        const FluentTimeResource& ftrA = requirements[a];

        std::map<temporal::Interval, Set> concurrent = { { ftrA.getInterval(), { ftrA }} };

        for(size_t b = a + 1; b < requirements.size(); ++b)
        {
            std::map<temporal::Interval, Set>::iterator it =
                concurrent.begin();
            for(; it != concurrent.end(); ++it)
            {
                temporal::Interval interval = it->first;
                Set& ftrs = it->second;

                const FluentTimeResource& ftrB = requirements[b];
                TimePoint::Ptr otherFrom = ftrB.getInterval().getFrom();
                TimePoint::Ptr otherTo = ftrB.getInterval().getTo();

                if(tpc.hasIntervalOverlap(interval.getFrom(), interval.getTo(),
                            otherFrom,
                            otherTo))
                {
                    temporal::Interval overlapInterval = tpc.getIntervalOverlap(interval.getFrom(),
                            interval.getTo(),
                            otherFrom,
                            otherTo);
                    if(overlapInterval == interval)
                    {
                        ftrs.insert(ftrB);
                    } else {
                        Set newFtrs = ftrs;
                        newFtrs.insert(ftrB);
                        concurrent[overlapInterval] = newFtrs;
                    }
                }
            }
        }

        for(std::pair<temporal::Interval,Set> c : concurrent)
        {
            Set& ftrs = c.second;
            const temporal::Interval& interval = c.first;
            if(ftrs.size() > 1)
            {
                std::cout << "Found overlap for: "
                    << interval.toString(4,true) << std::endl
                    << FluentTimeResource::toQualificationString(ftrs.begin(),
                            ftrs.end(), 4)
                    << std::endl;
                overlapping.push_back(ftrs);
            }
        }
    }
    return overlapping;
}

FluentTimeResource::List FluentTimeResource::createNonOverlappingRequirements(const List& _requirements,
        temporal::point_algebra::TimePoint::PtrList sortedTimepoints,
        temporal::point_algebra::TimePointComparator tpc
        )
{
    List newRequirements;
    List requirements = _requirements;
    sortForEarlierStart(requirements, tpc);

    for(size_t i = 0; i < sortedTimepoints.size()-1; ++i)
    {
        FluentTimeResource ftr;
        ftr.setOrganizationModelAsk(requirements.back().getOrganizationModelAsk());
        ftr.setLocation(requirements.back().getLocation());
        temporal::Interval interval(sortedTimepoints[i],
                sortedTimepoints[i+1],
                tpc);

        ftr.setInterval(interval);

        for(const FluentTimeResource& r : requirements)
        {
            if(interval.overlaps(r.getInterval()))
            {
                ftr.merge(r);
            }
        }
        if(!ftr.getMinCardinalities().empty()
                || !ftr.getMaxCardinalities().empty())
        {
            newRequirements.push_back(ftr);
        }
    }
    return newRequirements;
}

bool FluentTimeResource::hasFunctionalityConstraint() const
{
    for(const organization_model::Resource& resource : mRequiredResources)
    {
        if( mOrganizationModelAsk.ontology().isSubClassOf(resource.getModel(),
                    organization_model::vocabulary::OM::Functionality()))
        {
            return true;
        }
    }
    return false;
}

organization_model::Resource::Set FluentTimeResource::getRequiredResources() const
{
    return mRequiredResources;
}

void FluentTimeResource::addRequiredResource(const organization_model::Resource& resource)
{
    mResources.insert(resource.getModel());
    if(mOrganizationModelAsk.ontology().isSubClassOf(resource.getModel(),
                    organization_model::vocabulary::OM::Functionality()))
    {
        mRequiredResources.insert(resource);
        mRequiredResources = organization_model::Resource::merge(mRequiredResources);
    }
}

void FluentTimeResource::compact(std::vector<FluentTimeResource>& requirements)
{
    std::vector<FluentTimeResource>::iterator it = requirements.begin();
    for(; it != requirements.end(); ++it)
    {
        FluentTimeResource& fts = *it;

        std::vector<FluentTimeResource>::iterator compareIt = it + 1;
        for(; compareIt != requirements.end();)
        {
            FluentTimeResource& otherFts = *compareIt;

            if(fts.mTimeInterval == otherFts.mTimeInterval &&
                    fts.mpLocation == otherFts.mpLocation)
            {
                fts.merge(otherFts);
                requirements.erase(compareIt);
            } else {
                ++compareIt;
            }
        }
    }
}

void FluentTimeResource::merge(const FluentTimeResource& otherFtr)
{
    mResources.insert(otherFtr.mResources.begin(), otherFtr.mResources.end());
    mRequiredResources.insert(otherFtr.mRequiredResources.begin(),
            otherFtr.mRequiredResources.end());
    mRequiredResources = organization_model::Resource::merge(mRequiredResources);

    mMinCardinalities = organization_model::Algebra::max(mMinCardinalities,
            otherFtr.mMinCardinalities);
    mMaxCardinalities = organization_model::Algebra::min(mMaxCardinalities,
            otherFtr.mMaxCardinalities);
}

organization_model::ModelPool::Set FluentTimeResource::getDomain() const
{
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
    organization_model::ModelPool::Set combinations = mOrganizationModelAsk.getResourceSupport(requiredResources);

    LOG_INFO_S << "Support for the following resource requested:" << std::endl
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
        if(ftr.mTimeInterval == fluent.mTimeInterval &&
                ftr.mpLocation == fluent.mpLocation)
        {
            owlapi::model::IRISet intersect;
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
    mResources.insert(model);
    mMinCardinalities[model] += number;
}

bool FluentTimeResource::areMutualExclusive(const FluentTimeResource& ftrA,
            const FluentTimeResource& ftrB,
            temporal::point_algebra::TimePointComparator tpc)
{
    if(ftrA.getLocation() != ftrB.getLocation())
    {
        const temporal::Interval& intervalA = ftrA.getInterval();
        const temporal::Interval& intervalB = ftrB.getInterval();

        // either interval a prece
        if(intervalA.before(intervalB, tpc)
                || intervalB.before(intervalA, tpc))
        {
            return false;
        } else {
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
        const temporal::Interval& intervalA = ftrA.getInterval();
        const temporal::Interval& intervalB = ftrB.getInterval();

        return intervalA.overlaps(intervalB, tpc);
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

std::string FluentTimeResource::toQualificationString(const List& list, uint32_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    for(const FluentTimeResource& ftr : list)
    {
        ss << hspace << ftr.getQualificationString() << std::endl;
    }
    return ss.str();
}

void FluentTimeResource::updateIndices(const symbols::constants::Location::PtrList& locations)
{
    symbols::constants::Location::PtrList::const_iterator lit =
        std::find(locations.begin(), locations.end(), mpLocation);
    if(lit != locations.end())
    {
        mFluentIdx = std::distance(locations.begin(), lit);
    } else {
        throw
            std::invalid_argument("templ::solvers::FluentTimeResource::updateIdxs:"
                    " could not find a location named '" + mpLocation->toString()
                    + "'");
    }
}

void FluentTimeResource::updateIndices(List& requirements,
        const symbols::constants::Location::PtrList& locations)
{
    for(FluentTimeResource& ftr : requirements)
    {
        ftr.updateIndices(locations);
    }
}

} // end namespace solvers
} // end namespace templ
