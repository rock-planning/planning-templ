#ifndef TEMPL_SOLVERS_CSP_TEMPORALLY_EXPANDED_GRAPH_HPP
#define TEMPL_SOLVERS_CSP_TEMPORALLY_EXPANDED_GRAPH_HPP

#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>


namespace templ {
namespace solvers {
namespace csp {

/**
 * Check if a given array of the form [srcTimepointFluentIdx*#timepointFluents + targetTimepointFluentIdx]
 * is representing a path that conforms to temporal constraints
 */
class IsPath : public Gecode::NaryPropagator<Gecode::Int::IntView,Gecode::Int::PC_INT_BND>
{
public:
    typedef Gecode::ViewArray<Gecode::Int::IntView> IntVarArrayView;

protected:
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;

    uint32_t mNumberOfVertices;
    IntVarArrayView mGraph;

    static Gecode::LinIntExpr sumOfArray(const Gecode::ViewArray<Gecode::Int::IntView>& view, uint32_t from, uint32_t n);

public:
    /**
     * Spans an temporally extended network of size <numberOfTimepoints> X <numberOfFluents>
     * Checks if the given IntVarArray forms a path
     */
    IsPath(Gecode::Space& home, IntVarArrayView& graph, uint32_t numberOfTimepoints, uint32_t numberOfFluents);

    IsPath(Gecode::Space& home, bool share, IsPath& p);

    static Gecode::ExecStatus post(Gecode::Space& home, IntVarArrayView& x0, uint32_t numberOfTimepoints, uint32_t numberOfFluents);

    /**
     * Cancels that subscription of the view
     * \return the size of the just disposed propagator
     */
    virtual size_t dispose(Gecode::Space& home);

    virtual Gecode::Propagator* copy(Gecode::Space& home, bool share);

    virtual Gecode::PropCost cost(const Gecode::Space&, const Gecode::ModEventDelta&) const;

    virtual Gecode::ExecStatus propagate(Gecode::Space& home, const Gecode::ModEventDelta&);
};

void isPath(Gecode::Space& home, const Gecode::IntVarArgs&, uint32_t numberOfTimepoints, uint32_t numberOfFluents);

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TEMPORALLY_EXPANDED_GRAPH_HPP
