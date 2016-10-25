#ifndef TEMPL_SOLVERS_CSP_PROPAGATORS_IS_PATH_HPP
#define TEMPL_SOLVERS_CSP_PROPAGATORS_IS_PATH_HPP

#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>


namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

/**
 * Check if a given array of the form [srcTimepointFluentIdx*#timepointFluents + targetTimepointFluentIdx]
 * is representing a path that conforms to temporal constraints
 */
class IsPath : public Gecode::NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_ANY>
{
public:
    typedef Gecode::ViewArray<Gecode::Set::SetView> SetVarArrayView;

protected:
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;

    uint32_t mNumberOfVertices;
    SetVarArrayView mGraph;

public:
    /**
     * Spans a temporally extended network of size <numberOfTimepoints> X <numberOfFluents>
     * Checks if the given SetVarArray forms a path
     */
    IsPath(Gecode::Space& home, SetVarArrayView& graph, uint32_t numberOfTimepoints, uint32_t numberOfFluents);

    IsPath(Gecode::Space& home, bool share, IsPath& p);

    static Gecode::ExecStatus post(Gecode::Space& home, SetVarArrayView& x0, uint32_t numberOfTimepoints, uint32_t numberOfFluents, uint32_t minPathLength = 1, uint32_t maxPathLength = std::numeric_limits<uint32_t>::max());

    /**
     * Cancels that subscription of the view
     * \return the size of the just disposed propagator
     */
    virtual size_t dispose(Gecode::Space& home);

    virtual Gecode::Propagator* copy(Gecode::Space& home, bool share);

    virtual Gecode::PropCost cost(const Gecode::Space&, const Gecode::ModEventDelta&) const;

    virtual Gecode::ExecStatus propagate(Gecode::Space& home, const Gecode::ModEventDelta&);
};

void isPath(Gecode::Space& home, const Gecode::SetVarArgs&, uint32_t numberOfTimepoints, uint32_t numberOfFluents, uint32_t minPathLength = 1, uint32_t maxPathLength = std::numeric_limits<uint32_t>::max());

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_PROPAGATORS_IS_PATH_HPP
