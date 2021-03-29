#ifndef TEMPL_SOLVERS_CSP_CONTEXT_HPP
#define TEMPL_SOLVERS_CSP_CONTEXT_HPP

#include "../../Mission.hpp"
#include <qxcfg/Configuration.hpp>

namespace templ {
namespace solvers {
namespace csp {

class Context
{
public:
    typedef shared_ptr<Context> Ptr;

    Context(const Mission::Ptr& mission, const qxcfg::Configuration& configuration = qxcfg::Configuration());

    moreorg::OrganizationModelAsk& ask() { return mAsk; }

    const std::vector<symbols::constants::Location::Ptr>& locations() const { return mLocations; }

    const std::vector<solvers::temporal::Interval>& intervals() const { return mIntervals; }

    size_t getNumberOfTimepoints() const { return mNumberOfTimepoints; }
    size_t getNumberOfFluents() const { return mNumberOfFluents; }

    const qxcfg::Configuration& configuration() const { return mConfiguration; }



private:
    moreorg::OrganizationModelAsk mAsk;

    /// Constants: Locations (as defined in the mission)
    std::vector<symbols::constants::Location::Ptr> mLocations;

    std::vector<solvers::temporal::Interval> mIntervals;

    /// Configuration object
    qxcfg::Configuration mConfiguration;

    size_t mNumberOfTimepoints;
    size_t mNumberOfFluents;
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_CONTEXT_HPP
