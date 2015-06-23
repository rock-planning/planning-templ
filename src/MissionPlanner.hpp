#ifndef TEMPL_MISSION_PLANNER_HPP
#define TEMPL_MISSION_PLANNER_HPP

#include <vector>
#include <templ/solvers/temporal/Chronicle.hpp>
#include <templ/Mission.hpp>
#include <organization_model/OrganizationModelAsk.hpp>

namespace templ {

typedef std::vector<Mission> CandidateMissions;

class MissionPlanner
{
    typedef std::map<StateVariable, std::vector< std::vector<StateVariable> >  >
        StateVariableExpansionMap;

    StateVariableExpansionMap mStateVariableExpansionMap;

public:
    MissionPlanner(const Mission& mission);

    CandidateMissions solve();

protected:
    solvers::temporal::Chronicle::Ptr getCandidateChronicle();
    void computeExpansion(const StateVariable& stateVariable);

    Mission mCurrentMission;
    organization_model::OrganizationModelAsk mOrganizationModelAsk;
};

} // end namespace templ
#endif // TEMPL_MISSION_PLANNER_HPP
