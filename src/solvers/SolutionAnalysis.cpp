#include "SolutionAnalysis.hpp"

namespace templ {
namespace solvers {

SolutionAnalysis::SolutionAnalysis(const Mission::Ptr& mission, const Solution& solution)
    : mpMission(mission)
    , mSolution(solution)
{
}

void SolutionAnalysis::analyse()
{
    // a collect all requirements of the mission
    std::vector<solvers::csp::FluentTimeResource> requirements = getResourceRequirements(mission);

    for(const solvers::csp::FluentTimeResource& ftr : requirements)
    {
        analyse(ftr);
    }
}

void SolutionAnalysis::analyse(const solvers::csp::FluentTimeResource& ftr)
{
    // Main spatio temporal requirement
    // needs to be checked upon fulfillment
    //
    // -- partial fulfillment or full fulfillment --
    std::vector<Modelsolvers::csp::FluentTimeResource& ftr solution.collectRelated(ftr);

}

} // end namespace solvers
} // end namespace templ
