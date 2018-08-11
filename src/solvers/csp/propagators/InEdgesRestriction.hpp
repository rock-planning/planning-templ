#ifndef TEMPL_SOLVERS_CSP_PROPAGATORS_IN_EDGES_RESTRICTION_HPP
#define TEMPL_SOLVERS_CSP_PROPAGATORS_IN_EDGES_RESTRICTION_HPP

#include <gecode/int.hh>
#include <gecode/set.hh>
#include <set>
#include <map>

#include "Idx.hpp"

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

/**
 * Make sure the location access is restricted
 */
class InEdgesRestriction : public Gecode::NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_NONE>
{
public:
    Gecode::Council<Idx> c;
    std::set<int> mAssignedTimepointIndices;


protected:
    std::string mTag;
    size_t mNumberOfTimelines;
    size_t mNumberOfTimepoints;
    size_t mNumberOfFluents;
    size_t mLocationIdx;
    size_t mMinCount;
    size_t mMaxCount;

    // count the already assigned
    std::map<int, size_t> mAssignedTimepoints;

public:
    /**
     * Spans a temporally extended network of size <numberOfTimepoints> X <numberOfFluents>
     * Checks if the given SetVarArray forms a path
     */
    InEdgesRestriction(Gecode::Space& home, SetVarArrayView& graph,
            size_t numberOfTimelines,
            size_t numberOfTimepoints, size_t numberOfFluents,
            size_t locationIdx,
            size_t minCount, size_t maxCount,
            const std::string& tag = "");

    InEdgesRestriction(Gecode::Space& home, InEdgesRestriction& p);

    /**
     * InEdgesRestriction propagators post function, i.e. when it is initially created
     */
    static Gecode::ExecStatus post(Gecode::Space& home,
        const std::vector<Gecode::SetVarArray>& xArray,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        size_t locationIdx,
        size_t minCount, size_t maxCount,
        const std::string& tag = "");

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
};

/**
 * Restrict number of in edges
 */
void restrictInEdges(Gecode::Space& home, const std::vector<Gecode::SetVarArray>&,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        size_t locationIdx,
        size_t minCount, size_t maxCount,
        const std::string& tag = "");

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif //TEMPL_SOLVERS_CSP_PROPAGATORS_IN_EDGES_RESTRICTION_HPP
