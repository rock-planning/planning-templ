#include "TimelineBrancher.hpp"
#include "../TransportNetwork.hpp"

namespace templ {
namespace solvers {
namespace csp {

size_t TimelineBrancher::getChoiceSize(Gecode::Space& space) const
{
    TransportNetwork& network = static_cast<TransportNetwork&>(space);
    size_t numberOfFluents = network.getNumberOfFluents();
    size_t numberOfRoles = network.getActiveRoleList().size();

    // Upper bound on choice
    return numberOfFluents*numberOfFluents* numberOfRoles ;
}

void branchTimelines(Gecode::Home home, const Gecode::SetVarArgs& x)
{
    if(home.failed())
    {
        return;
    }
    TimelineBrancher::TimelineView y(home, x);
    TimelineBrancher::post(home, y);
}

} // end namespace templ
} // end namespace solvers
} // end namespace csp
