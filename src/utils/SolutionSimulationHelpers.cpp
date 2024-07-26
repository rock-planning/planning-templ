#include "SolutionSimulationHelpers.hpp"
#include <moreorg/metrics/pdfs/WeibullPDF.hpp>
#include <moreorg/reasoning/ResourceMatch.hpp>
#include <moreorg/reasoning/ResourceInstanceMatch.hpp>
#include <moreorg/metrics/ProbabilityOfFailure.hpp>
#include "../solvers/Cost.hpp"

using namespace moreorg;

namespace templ {
namespace utils { 

solvers::SolutionAnalysis::MinMaxModelPools getRequiredResources(
        const solvers::FluentTimeResource& ftr,
        const std::vector<solvers::FluentTimeResource> &resourceRequirements,
        const OrganizationModelAsk& om)
{
    using namespace solvers::temporal::point_algebra;
    solvers::SolutionAnalysis::MinMaxModelPools minMaxModelPools;

    std::vector<solvers::FluentTimeResource>::const_iterator cit = resourceRequirements.begin();
    for (; cit != resourceRequirements.end(); ++cit)
    {
        const solvers::FluentTimeResource &requirementFtr = *cit;
        if (ftr.getLocation() == requirementFtr.getLocation() &&
            ftr.getInterval() == requirementFtr.getInterval())
        {
            ModelPool minCardinalities = ftr.getMinCardinalities();
            for (const Resource &resource : ftr.getRequiredResources())
            {
                // Assuming the functionalities have max
                if (om.ontology().isSubClassOf(resource.getModel(),
                                               vocabulary::OM::Functionality()))
                {
                    minCardinalities[resource.getModel()] = 1;
                }
            }

            minMaxModelPools.first.push_back(minCardinalities);
            minMaxModelPools.second.push_back(ftr.getMaxCardinalities());
            return minMaxModelPools;
        }
    }

    throw std::invalid_argument("templ::solvers::SolutionAnalysis::getRequiredResources: could not find the corrensponding requirement for an existing fluent time resource");
}

moreorg::ModelPool getMinResourceRequirements(
    const solvers::FluentTimeResource& ftr,
    const std::vector<solvers::FluentTimeResource>& resourceRequirements,
    const moreorg::OrganizationModelAsk& om)
{
    return getRequiredResources(ftr, resourceRequirements, om).first.front();
}

moreorg::ResourceInstance::List getMinAvailableResources(
    const SpaceTime::Network::tuple_t::Ptr& tuple,
    const SpaceTime::Network& solutionNetwork,
    const moreorg::OrganizationModelAsk& om)
{
    moreorg::ResourceInstance::List availableResources = getAvailableResources(tuple->first(), tuple->second(), solutionNetwork, om);

    //    // Infer functionality from this set of resources
    //    OrganizationModelAsk ask(mpMission->getOrganizationModel(),
    //            minAvailableResources,
    //            true);
    //    // Creating model pool from available functionalities
    //    ModelPool functionalities = ask.getSupportedFunctionalities();
    //    return moreorg::Algebra::max(minAvailableResources, functionalities);
    return availableResources;
}

ResourceInstance::List getAvailableResources(
    const symbols::constants::Location::Ptr& location,
    const solvers::temporal::point_algebra::TimePoint::Ptr& timepoint,
    const SpaceTime::Network& solutionNetwork,
    const moreorg::OrganizationModelAsk& om)
{
    ResourceInstance::List availableAgents;
    // identified relevant tuple
    try
    {
        SpaceTime::Network::tuple_t::Ptr tuple = solutionNetwork.tupleByKeys(location, timepoint);
        Role::Set foundRoles = tuple->getRoles(RoleInfo::ASSIGNED);
        Role::List roles(foundRoles.begin(), foundRoles.end());
        for (const auto& role : roles)
        {
            ResourceInstance::List available = om.getRelated(role, vocabulary::OM::Resource(), vocabulary::OM::has(), false);
            availableAgents.insert(std::end(availableAgents), std::begin(available), std::end(available));
        }
    }
    catch (const std::exception &e)
    {
        LOG_WARN_S << e.what();
    }
    return availableAgents;
}

metrics::Probability::List initializeProbabilityList(
    const ProbabilityType &type,
    std::map<reasoning::ModelBound, ResourceInstance::List>& resourceAssignmentMap,
    ResourceInstance::List& available,
    const moreorg::OrganizationModelAsk& ask,
    double t0,
    double t1)
{
    using namespace moreorg::metrics;
    switch (type)
    {
        case POF:
        {
            ProbabilityOfFailure::List pofList;
            ProbabilityDensityFunction::Ptr defaultPDF = make_shared<pdfs::WeibullPDF>(144000., 1.);
            for (const auto& reqAssignmentPair : resourceAssignmentMap)
            {
                ProbabilityDensityFunction::Ptr probabilityDensityFunction;
                try
                {
                    probabilityDensityFunction = ProbabilityDensityFunction::getInstance(ask, reqAssignmentPair.first.model);
                }
                catch (...)
                {
                    probabilityDensityFunction = defaultPDF;
                }

                ProbabilityOfFailure pof(reqAssignmentPair.first, reqAssignmentPair.second, probabilityDensityFunction);
                pofList.push_back(pof);
            }

            bool updated = false;
            do
            {
                updated = false;
                // Sort based on probability of survival -- try to maximize redundancy
                std::sort(
                    pofList.begin(), pofList.end(),
                    [t0, t1](const Probability &a, const Probability &b)
                    {
                        return static_cast<ProbabilityOfFailure>(a).getProbabilityOfSurvivalConditionalWithRedundancy(t0, t1) <
                               static_cast<ProbabilityOfFailure>(b).getProbabilityOfSurvivalConditionalWithRedundancy(t0, t1);
                    });

                ResourceInstance::List::iterator rit = available.begin();
                for (; rit != available.end(); ++rit)
                {
                    ResourceInstance &remaining = *rit;
                    bool hasPossibleMatch = false;

                    // Try to fit remaining resources
                    for (auto survivability : pofList)
                    {
                        // Check if model can be used to strengthen the survivability
                        if (survivability.getQualification() == remaining.getModel() ||
                            ask.ontology().isSubClassOf(
                                remaining.getModel(), survivability.getQualification()))
                        {
                            hasPossibleMatch = true;
                            try
                            {
                                survivability.addAssignment(remaining);
                                available.erase(rit);
                                --rit;
                                updated = true;
                                break;
                            }
                            catch (...)
                            {
                                available.erase(rit);
                                --rit;
                                break;
                            }
                        }
                    }
                    if (!hasPossibleMatch)
                    {
                        available.erase(rit);
                        --rit;
                    }
                }
            } while (updated);
            return pofList;
        }
        default:
        {
            throw std::invalid_argument("proactive_planning::utils::initializeProbabilityList: no valid probability Type was given!");
            break;
        }
    }
}

metrics::Probability::List matchResourcesToProbability(
    const ProbabilityType& type,
    const moreorg::OrganizationModelAsk &ask,
    const std::vector<owlapi::model::OWLCardinalityRestriction::Ptr>& required,
    ResourceInstance::List& availableAgents,
    double t0,
    double t1)
{
    if (required.empty())
    {
        throw std::invalid_argument("proactive_planning::utils::matchResourcesToProbability: set of cardinality restriction to "
                                    "define requirements is empty");
    }
    using namespace moreorg::metrics;
    using namespace moreorg::reasoning;
    ModelBound::List requiredModelBound = ResourceInstanceMatch::toModelBoundList(required);
    ResourceInstance::List available(availableAgents); // ?
    std::map<ModelBound, ResourceInstance::List> assignments;
    if (!ResourceMatch::hasMinRequirements(requiredModelBound))
    {
        throw std::invalid_argument(
            "moreorg::metrics::Redundancy: model bound requires minimum "
            "requirements"
            "to complete redundancy computation, but none are provided");
    }
    ResourceInstanceMatch::Solution solution;
    int fullModelRedundancy = 0;
    try
    {
        while (true)
        {
            solution = ResourceInstanceMatch::solve(requiredModelBound, available, ask);
            ++fullModelRedundancy; // use this to check whether all requierements got matches
            for (ModelBound model : requiredModelBound)
            {
                ResourceInstance::List currentAssignments = solution.getAssignments(model.model);
                assignments[model].insert(std::end(assignments[model]), std::begin(currentAssignments), std::end(currentAssignments));
            }
            available = solution.removeAssignmentsFromList(available);
        }
    }
    catch (const std::exception &e)
    {
        LOG_DEBUG_S << "ResourceInstanceMatch failed: " << e.what();
    }

    if (fullModelRedundancy == 0)
    {
        // LOG_WARN_S << "matchResourcesToProbaiblity: the minimal resource requirements have not been "
        //               "fulfilled. Cannot match"
        //            << "available: " << ModelBound::toString(modelBoundRemaining, 4)
        //            << "required: " << ModelBound::toString(requiredModelBound, 4);
        LOG_WARN_S << "Could not assign available Resources to all requirements: "
                   << "required: " << ModelBound::toString(requiredModelBound, 4);
        throw std::runtime_error("proactive_planning::utils::matchResourcesToProbability: minimal resource "
                                 "requirement have not been fulfilled");
    }
    metrics::Probability::List probabilityModels = initializeProbabilityList(type, assignments, available, ask, t0, t1);

    return probabilityModels;
}

solvers::temporal::TemporalConstraintNetwork::Assignment quantifyTime(
    const templ::Mission::Ptr& mission,
    const SpaceTime::Network& solutionNetwork,
    std::vector<solvers::FluentTimeResource>&resourceRequirements)
{
    using namespace solvers::temporal;
    TemporalConstraintNetwork tcn;
    solvers::Cost cost(mission->getOrganizationModelAsk());

    // set min duration for requirements
    for (const solvers::FluentTimeResource &ftr : resourceRequirements)
    {
        IntervalConstraint::Ptr intervalConstraint = make_shared<IntervalConstraint>(ftr.getInterval().getFrom(), ftr.getInterval().getTo());
        // TODO add reading these bounds from ontology (maybe)
        Bounds bounds(1800, std::numeric_limits<double>::max());
        intervalConstraint->addInterval(bounds);
        tcn.addIntervalConstraint(intervalConstraint);
    }
    // Apply constraints from the current solution
    // TODO: space time network: iterator over all 'solution' edges
    using namespace graph_analysis;
    graph_analysis::EdgeIterator::Ptr edgeIt = solutionNetwork.getGraph()->getEdgeIterator();
    while (edgeIt->next())
    {

        SpaceTime::Network::edge_t::Ptr edge = dynamic_pointer_cast<SpaceTime::Network::edge_t>(edgeIt->current());
        if (!edge)
        {
            throw std::invalid_argument("SolutionAnalysisFunctionalities: encountered edge type: " + edgeIt->current()->getClassName() + " Are you loading final plan instead of the transport network solution?");
        }

        Role::Set roles = edge->getRoles(RoleInfo::ASSIGNED);
        if (roles.empty())
        {
            continue;
        }

        SpaceTime::Network::tuple_t::Ptr sourceTuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(edge->getSourceVertex());
        SpaceTime::Network::tuple_t::Ptr targetTuple = dynamic_pointer_cast<SpaceTime::Network::tuple_t>(edge->getTargetVertex());

        symbols::constants::Location::Ptr sourceLocation = sourceTuple->first();
        symbols::constants::Location::Ptr targetLocation = targetTuple->first();

        double distanceInM = cost.getTravelDistance({sourceLocation, targetLocation});
        double minTravelTime = cost.estimateTravelTime(sourceLocation, targetLocation, roles);
        LOG_WARN_S << "Estimated travelTime: " << minTravelTime << " for " << Role::toString(roles)
                   << "    from: " << sourceLocation->toString() << "/" << sourceTuple->second()->toString() << std::endl
                   << "    to: " << targetLocation->toString() << "/" << targetTuple->second()->toString() << std::endl
                   << "    estimated travelDistance in m: " << distanceInM;

        point_algebra::TimePoint::Ptr sourceTimepoint = sourceTuple->second();
        point_algebra::TimePoint::Ptr targetTimepoint = targetTuple->second();

        IntervalConstraint::Ptr intervalConstraint =
            make_shared<IntervalConstraint>(sourceTimepoint, targetTimepoint);

        // Compute the required transition time
        RoleInfo::Ptr roleInfo = dynamic_pointer_cast<RoleInfo>(sourceTuple);
        double reconfigurationCost = 0;
        try
        {
            reconfigurationCost = roleInfo->getAttribute(RoleInfo::RECONFIGURATION_COST);
        }
        catch (const std::exception &e)
        {
            LOG_WARN_S << "No reconfiguration cost for " << roleInfo->toString(4);
        }

        double requiredTime = minTravelTime + reconfigurationCost;
        if (requiredTime > 0)
        {
            Bounds bounds(requiredTime, std::numeric_limits<double>::max());
            intervalConstraint->addInterval(bounds);
            tcn.addIntervalConstraint(intervalConstraint);

            LOG_INFO_S << "Interval constraint between: " << sourceTimepoint->toString() << " and " << targetTimepoint->toString() << " min travel time: "
                       << minTravelTime << ", requiredTime (inkl. reconfiguration) " << requiredTime;
        }
    }

    for (Constraint::Ptr c : mission->getConstraints())
    {
        IntervalConstraint::Ptr ic = dynamic_pointer_cast<IntervalConstraint>(c);
        if (ic)
        {
            Edge::PtrList edges = tcn.getDistanceGraph()->getEdges(ic->getSourceVertex(), ic->getTargetVertex());
            if (edges.empty())
            {
                tcn.addIntervalConstraint(ic);
            }
            else
            {
                IntervalConstraint::Ptr existingIc = dynamic_pointer_cast<IntervalConstraint>(edges.front());
                existingIc->appendBounds(*ic);
            }
        }
    }

    //graph_analysis::io::GraphIO::write("/tmp/distanceGraph", tcn.getDistanceGraph(), graph_analysis::representation::GEXF);
    //graph_analysis::io::GraphIO::write("/tmp/distanceGraph", tcn.getDistanceGraph(), graph_analysis::representation::GRAPHVIZ);

    tcn.stpWithConjunctiveIntervals();
    tcn.stp();
    tcn.minNetwork();

    TemporalConstraintNetwork::Assignment timeAssignment = tcn.getAssignment();
    return timeAssignment;
}

ResourceInstance::List filterResourcesByBlacklist(
    ResourceInstance::List& resources,
    ResourceInstance::List& blacklist)
{
    ResourceInstance::List returnList;
    for (auto &r : resources)
    {
        auto it = std::find_if(blacklist.begin(), blacklist.end(), [&r](ResourceInstance assignment)
                               { return ((assignment.getName() == r.getName()) && (assignment.getAtomicAgent() == r.getAtomicAgent())); });
        if (it == blacklist.end())
        {
            returnList.push_back(r);
        }
    }
    return returnList;
}

ModelPool checkAndRemoveRequirements(
    ModelPool& available,
    ModelPool& required,
    const moreorg::OrganizationModelAsk& ask)
{
    ModelPool returnModelPool;
    for (auto &r : required)
    {
        ModelPool tempModelPool;
        tempModelPool.setResourceCount(r.first, r.second);
        std::vector<owlapi::model::OWLCardinalityRestriction::Ptr> r_required = ask.getRequiredCardinalities(tempModelPool, vocabulary::OM::has());
        bool check = true;
        for (auto &rr : r_required)
        {
            bool found = false;
            owlapi::model::OWLObjectCardinalityRestriction::Ptr restriction = dynamic_pointer_cast<owlapi::model::OWLObjectCardinalityRestriction>(rr);
            for (auto &a : available)
            {
                if ((a.first == restriction->getQualification()) || (ask.ontology().isSubClassOf(a.first, restriction->getQualification())))
                {
                    // TODO what about SuperClass Requirements and multiple Sublasses are available ? Make counter?
                    if (a.second < restriction->getCardinality())
                    {
                        check = false;
                        break;
                    }
                    else
                    {
                        found = true;
                        break;
                    }
                }
            }
            if (!found)
            {
                check = false;
                break;
            }
        }
        if (check)
        {
            returnModelPool.setResourceCount(r.first, r.second);
        }
    }
    return returnModelPool;
}

ResourceInstance::List filterSingleResource(
    ResourceInstance::List& resources,
    ResourceInstance& resourceToRemove)
{
    ResourceInstance::List returnList;
    for (auto &r : resources)
    {
        if (resourceToRemove.getName() != r.getName())
        {
            returnList.push_back(r);
        }
    }
    return returnList;
}

int checkFutureImpactOfAssignment(
    SpaceTime::Network& solution,
    const templ::Mission::Ptr& mission,
    ResourceInstance& assignment,
    ResourceInstance::List& previouslyFailed,
    SpaceTime::Network::tuple_t::Ptr& vertexTuple,
    std::map<SpaceTime::Network::tuple_t::Ptr, solvers::FluentTimeResource::List>& tupleFtrMap,
    OrganizationModelAsk& omAsk,
    const std::vector<solvers::FluentTimeResource>& resourceRequirements)
{
    solvers::Solution newSolution(solution, mission->getOrganizationModel());
    SpaceTime::Route roleRoute = newSolution.getRoute(assignment.getAtomicAgent());
    bool foundTuple = false;
    int missedRequirement = 0;
    for (auto &tuple : roleRoute)
    {
        if ((tuple->first() == vertexTuple->first()) && (tuple->second() == vertexTuple->second()) && (!foundTuple))
        {
            foundTuple = true;
            continue;
        }

        if (foundTuple)
        {
            ModelPool required;
            for (const solvers::FluentTimeResource &ftr : tupleFtrMap[tuple])
            {
                if (required.empty())
                {
                    required = getMinResourceRequirements(ftr, resourceRequirements, omAsk);
                }
                else
                {
                    ModelPool minRequired = getMinResourceRequirements(ftr, resourceRequirements, omAsk);
                    Algebra::merge(minRequired, required);
                }
            }
            if (required.empty())
                continue;

            ResourceInstance::List availableUnfiltered = getMinAvailableResources(tuple, solution, omAsk);
            ResourceInstance::List available = utils::filterResourcesByBlacklist(availableUnfiltered, previouslyFailed);
            ModelPool availableModelPool = ResourceInstance::toModelPool(available);
            // std::cout << "required: " << required.toString() << std::endl;
            // std::cout << "available: " << availableModelPool.toString() << std::endl;

            ModelPool required_filtered = checkAndRemoveRequirements(availableModelPool, required, omAsk);

            missedRequirement += ((int)required.size() - (int)required_filtered.size());
        }
    }
    return missedRequirement;
}

} // end namespace utils
} // end namespace templ
