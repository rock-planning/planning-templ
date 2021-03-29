#include "Context.hpp"
#include <moreorg/Algebra.hpp>
#include <moreorg/vocabularies/OM.hpp>

namespace templ {
namespace solvers {
namespace csp {

Context::Context(const templ::Mission::Ptr& mission, const qxcfg::Configuration& configuration)
    : mAsk(mission->getOrganizationModel(),
            mission->getAvailableResources(), true,
            1000*configuration.getValueAs<double>("TransportNetwork/search/options/connectivity/timeout_in_s",20),
            configuration.getValue("TransportNetwork/search/options/connectivity/interface-type",moreorg::vocabulary::OM::ElectroMechanicalInterface().toString()))
    , mLocations(mission->getLocations())
    , mIntervals(mission->getTimeIntervals())
    , mConfiguration(configuration)
    , mNumberOfTimepoints(mission->getUnorderedTimepoints().size())
    , mNumberOfFluents(mLocations.size())
{
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ

