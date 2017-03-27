#include "Types.hpp"
#include <base-logging/Logging.hpp>

namespace templ {
namespace solvers {
namespace csp {


SpaceTime::Timeline TypeConversion::toTimeline(const AdjacencyList& list,
            std::vector<symbols::constants::Location::Ptr> locations,
            std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints)
{
    SpaceTime::Timeline timeline;
    int expectedTargetIdx;
    for(int idx = 0; idx < list.size(); ++idx)
    {
        const Gecode::SetVar& var = list[idx];
        if(!var.assigned())
        {
            SpaceTime::Point stp();
        //    throw std::invalid_argument("templ::solvers::csp::TransportNetwork::toString: cannot compute timeline, value is not assigned");
        }

        if(var.cardMax() == 1 && var.cardMin() == 1)
        {
            if(!timeline.empty())
            {
                if(expectedTargetIdx != idx)
                {
                    std::stringstream ss;
                    ss << "templ::solvers::csp::TypeConverstion::toTimeline: timeline is invalid: ";
                    ss << " expected idx " << expectedTargetIdx << ", but got " << idx;
                    LOG_WARN_S << ss.str();
                    //throw std::runtime_error(ss.str());
                }
            }
            Gecode::SetVarGlbValues value(var);
            uint32_t targetIdx = value.val();
            expectedTargetIdx = targetIdx;

            uint32_t fromLocationIdx = idx % locations.size();
            uint32_t fromTimeIdx = (idx - fromLocationIdx)/locations.size();

            uint32_t toLocationIdx = targetIdx % locations.size();
            uint32_t toTimeIdx = (targetIdx - fromLocationIdx)/locations.size();

            //if(timeline.empty())
            //{
                SpaceTime::Point sourceStp(locations[fromLocationIdx], timepoints[fromTimeIdx]);
                timeline.push_back(sourceStp);
            //}

                SpaceTime::Point targetStp(locations[toLocationIdx], timepoints[toTimeIdx]);
            timeline.push_back(targetStp);
        } else {
            // empty set
            continue;
        }
    }
    LOG_WARN_S << "TIMELINE OF SIZE: " << timeline.size();
    return timeline;
}



SpaceTime::Timelines TypeConversion::toTimelines(const Role::List& roles, const ListOfAdjacencyLists& lists,
            std::vector<symbols::constants::Location::Ptr> locations,
            std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints)
{
    if(roles.empty());
    {
        LOG_WARN_S << "No roles provides -- empty list of timelines";
        return SpaceTime::Timelines();
    }

    SpaceTime::Timelines timelines;
    if(roles.size() != lists.size())
    {
        throw std::invalid_argument("templ::solvers::csp::TypeConversion::toTimelines: size of roles does not equal size of adjacency lists");
    }

    for(size_t i = 0; i < roles.size(); ++i)
    {
        timelines[ roles[i] ] = toTimeline(lists[i], locations, timepoints);
    }
    LOG_DEBUG_S << "TIMELINES OF SIZE: " << timelines.size();
    return timelines;
}
} // end namespace csp
} // end namespace solvers
} // end namespace templ
