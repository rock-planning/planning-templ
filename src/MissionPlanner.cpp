#include "MissionPlanner.hpp"
#include <base/Logging.hpp>

namespace templ {

CandidateMissions MissionPlanner::solve(const Mission& mission)
{
    using namespace solvers;
    using namespace solvers::temporal;

    mCurrentMission = mission;

    //std::vector<PersistenceCondition::Ptr> persistenceConditions = mission.getPersistenceConditions();
    //std::vector<Constraint::Ptr> constraints = mission.getConstraints();

    mOrganizationModelAsk = OrganizationModelAsk(mCurrentMission.getOrganizationModel(),
            mission.getResources());


    std::vector<PersistenceCondition::Ptr>::const_iterator pit =
        persistenceConditions.begin();
    for(; pit != persistenceConditions.end(); ++pit)
    {
        PersistenceCondition::Ptr pc = *pit;
        StateVariable stateVariable = pc->getStateVariable();
        PlannerElement::Ptr plannerElement = pc->getValue();

        computeExpansion(stateVariable);
    }

    // How to plan
    // 1. - translate persistence condition from service model to minimally
    // required resources: check required models, add instance to domain
    //  e.g. 
    //    location_image_provider,m0 t0,t1
    //     --> sherpa,crex
    //    location_image_provider,m1 t1,t2
    //     --> sherpa,crex
    //
    //    - check if the constraint network remains consistent, how
    //       - assignment, check if rloc(sherpa)
    //        - get (minimal resource assignment) // functional saturation point
    //         - propagation(if possible)
    //         - check if timeline for rloc(sherpa), rloc(crex) ... remains
    //         consistent
    //         - if so: solution found
    //         - if not change instances in resource assignment / backtrack
    //
    //
    //     Create chronicle for the service based definition
    //     - translate into candidate chronicles for the resource based definition
    //       - service translates into (set of resources)
    //       - list of timelines, e.g. for rloc(sherpa_0),rloc(sherpa_1), ..., rloc(crex_0)
    //       - check consistency of chronicle
    //         - overlap of persistence condition
    //     
    //

    std::vector<Chronicle::Ptr> candidateChronicles;
    while(true)
    {
        Chronicle::Ptr chronicle = getCandidateChronicle();
        if(!chroncile)
        {
            break;
        } else if(chronicle->isConsistent())
        {
            LOG_INFO_S << "Solution found: " << chronicle->toString();
            candidateChronicles.push_back(chronicle);
        } else {
            LOG_INFO_S << "No solution found";
        }
    }

    return CandidateMissions();
}

void MissionPlanner::computeExpansion(const StateVariable& stateVariable)
{
    const std::string& function = stateVariable.getFunction();
    if(function == "service_location")
    {

    } else {
        throw std::invalid_argument("templ::MissionPlanner::computeExansion: could not expand state "
                "variable " + stateVariable.toString() + " requiring function 'service_location'");
    }
}

Chronicle::Ptr MissionPlanner::getCandidateChronicle()
{


}

} // end namespace templ
