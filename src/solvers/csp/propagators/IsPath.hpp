#ifndef TEMPL_SOLVERS_CSP_PROPAGATORS_IS_PATH_HPP
#define TEMPL_SOLVERS_CSP_PROPAGATORS_IS_PATH_HPP

#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>
#include <set>
#include "Idx.hpp"


namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

/**
 * Check if a given set is a consistent representation of a path
 * t0-l0 --> [ t1-l1, ... ]
 * t0-l1 --> [ t1-l1, ... ]
 */
class IsPath : public Gecode::NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_NONE>
{
public:

    Gecode::Council<Idx> c;

    // Queue all changes -- fluent and timepoint subscriptions
    // are parallel, so that we can identify the exact changed
    // entry by ('t','f') using the same index
    std::vector<int> mAssignedFluentIndices;
    std::vector<int> mAssignedTimepointIndices;


protected:
    std::string mTag;
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;

    uint32_t mMinPathLength;
    uint32_t mMaxPathLength;

    std::vector< std::pair<int, bool> > mAssignedTimepoints;

public:
    /**
     * Spans a temporally extended network of size <numberOfTimepoints> X <numberOfFluents>
     * Checks if the given SetVarArray forms a path
     */
    IsPath(Gecode::Space& home, SetVarArrayView& graph, const std::string& tag, uint32_t numberOfTimepoints, uint32_t numberOfFluents, int minPathLength = 1, int maxPathLength = Gecode::Int::Limits::max);

    IsPath(Gecode::Space& home, IsPath& p);

    /**
     * IsPath propagators post function, i.e. when it is initially created
     */
    static Gecode::ExecStatus post(Gecode::Space& home, const Gecode::SetVarArgs& x0, const std::string& tag, uint32_t numberOfTimepoints, uint32_t numberOfFluents, int minPathLength = 1, int maxPathLength = Gecode::Int::Limits::max);

    /**
     * Reduce domain of all possibly parallel edges
     */
    Gecode::ModEvent disableSametimeView(Gecode::Space& home, int viewIdx);

    /**
     * Set an upper bound for the domain of the set to a single value given by singleValueDomain
     * This allows to constrain outgoing edges to a single target node
     * \param singleValueDomain
     */
    Gecode::ModEvent constrainSametimeView(Gecode::Space& home, int viewIdx, int lowerBound, int upperBound);

    static bool isValidWaypointSequence(const std::vector< std::pair<int, bool> >& waypoints, size_t& startTimepoint, size_t& endTimepoint, bool fullyAssigned = false);

    /**
     * Cancels that subscription of the view
     * \return the size of the just disposed propagator
     */
    virtual size_t dispose(Gecode::Space& home);
    virtual Gecode::Propagator* copy(Gecode::Space& home);
    virtual Gecode::PropCost cost(const Gecode::Space&, const Gecode::ModEventDelta&) const;
    virtual void reschedule(Gecode::Space& home);
    virtual Gecode::ExecStatus advise(Gecode::Space& home, Gecode::Advisor& a, const Gecode::Delta& d);
    virtual Gecode::ExecStatus propagate(Gecode::Space& home, const Gecode::ModEventDelta&);

    std::string waypointsToString() const;

};

void isPath(Gecode::Space& home, const Gecode::SetVarArgs&,
        const std::string& tag,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents, int minPathLength = 1, int maxPathLength = Gecode::Int::Limits::max);

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_PROPAGATORS_IS_PATH_HPP
