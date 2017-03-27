#ifndef TEMPL_SOLVERS_CSP_TYPES_HPP
#define TEMPL_SOLVERS_CSP_TYPES_HPP

#include <vector>
#include <gecode/set.hh>
#include "../../SpaceTime.hpp"

namespace templ {
namespace solvers {
namespace csp {


typedef Gecode::SetVarArray AdjacencyList;
typedef std::vector<AdjacencyList> ListOfAdjacencyLists;

class TypeConversion
{
public:
    static SpaceTime::Timeline toTimeline(const AdjacencyList& list,
            std::vector<symbols::constants::Location::Ptr> locations,
            std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints
            );

    /**
     * Convert the adjacency lists of roles to timelines to timelines
     */
    static SpaceTime::Timelines toTimelines(const Role::List& roles, const ListOfAdjacencyLists& lists,
            std::vector<symbols::constants::Location::Ptr> locations,
            std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints);


};


} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TYPES_HPP
