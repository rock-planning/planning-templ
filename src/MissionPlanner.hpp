#ifndef TEMPL_MISSION_PLANNER_HPP
#define TEMPL_MISSION_PLANNER_HPP

#include <templ/Mission.hpp>

namespace templ {

typedef std::vector<Mission> CandidateMissions;

class MissionPlanner
{
    typedef std::map<StateVariable, std::vector< std::vector<StateVariable> > 
        StateVariableExpansionMap;

    StateVariableExpansionMap mStateVariableExpansionMap;

public:
    CandidateMissions solve(const Mission& mission);

protected:
    Chronicle::Ptr getCandidateChronicle();
    void computeExpansion(const StateVariable& stateVariable);

    Mission mCurrentMission;
    organization_model::OrganizationModelAsk mOrganizationModelAsk;
};

} // end namespace templ
#endif // TEMPL_MISSION_PLANNER_HPP
