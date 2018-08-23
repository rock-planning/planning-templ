#include "Types.hpp"
#include <base-logging/Logging.hpp>

namespace templ {
namespace solvers {
namespace csp {


SpaceTime::Timeline TypeConversion::toTimeline(const AdjacencyList& list,
            const std::vector<symbols::constants::Location::Ptr>& locations,
            const std::vector<solvers::temporal::point_algebra::TimePoint::Ptr>& timepoints,
            bool doThrow)
{
    SpaceTime::Timeline timeline;
    // find all timepoints
    for(size_t t = 0; t < timepoints.size();++t)
    {
        for(size_t f = 0; f < locations.size(); ++f)
        {
            size_t idx = t*locations.size() + f;

            const Gecode::SetVar& var = list[idx];
            if(!var.assigned())
            {
                if(doThrow)
                {
                    throw std::invalid_argument("templ::solvers::csp::TypeConversion::toTimeline: cannot compute timeline, value is not assigned");
                } else {
                    // skip this timepoint -- since it is not assigned yet
                    continue;
                }
            }
            // registration of source vertex
            if(var.lubSize() == 1)
            {
                symbols::constants::Location::Ptr currentLocation = locations[f];
                if(timeline.empty() || timeline.back().second != timepoints[t])
                {
                    SpaceTime::Point stp(locations[f], timepoints[t]);
                    timeline.push_back(stp);
                }

                // Add the target
                int target = var.lubMin();
                symbols::constants::Location::Ptr targetLocation = locations[target%locations.size()];
                SpaceTime::Point stp(targetLocation, timepoints[t+1]);
                timeline.push_back(stp);
                // found assignment so continue with next timepoint
                break;
            }
        }
    }
    return timeline;
}



SpaceTime::Timelines TypeConversion::toTimelines(const Role::List& roles, const ListOfAdjacencyLists& lists,
            const std::vector<symbols::constants::Location::Ptr>& locations,
            const std::vector<solvers::temporal::point_algebra::TimePoint::Ptr>& timepoints,
            bool doThrow)
{
    if(roles.empty())
    {
        return SpaceTime::Timelines();
    }

    SpaceTime::Timelines timelines;
    if(roles.size() != lists.size())
    {
        std::stringstream ss;
        ss << "templ::solvers::csp::TypeConversion::toTimelines: size of roles (" << roles.size() << ")";
        ss << " does not equal size of adjacency lists (" << lists.size() << ")";
        throw std::invalid_argument(ss.str());
    }

    for(size_t i = 0; i < roles.size(); ++i)
    {
        timelines[ roles[i] ] = toTimeline(lists[i], locations, timepoints, doThrow);
    }
    return timelines;
}
} // end namespace csp
} // end namespace solvers
} // end namespace templ
