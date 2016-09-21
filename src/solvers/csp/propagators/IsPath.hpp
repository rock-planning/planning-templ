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
class IsPath : public Gecode::NaryPropagator<Gecode::Int::IntView,Gecode::Int::PC_INT_BND>
{
public:
    typedef Gecode::ViewArray<Gecode::Int::IntView> IntVarArrayView;
    typedef std::pair<uint32_t, uint32_t> FluentTimeIdx;

protected:
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;

    uint32_t mNumberOfVertices;
    IntVarArrayView mGraph;

    /**
     * Compute the sum of the elements of a subsection of a IntView array
     * \param from Index to start from
     * \param n Offset defining the number of element to sum up from the given
     * from index
    */
    static Gecode::LinIntExpr sumOfArray(const Gecode::ViewArray<Gecode::Int::IntView>& view, uint32_t from, uint32_t n = 0);

    static Gecode::LinIntExpr sumOfMatrixSlice(const Gecode::ViewArray<Gecode::Int::IntView>& view, uint32_t fromCol, uint32_t fromRow, uint32_t toCol, uint32_t toRow, uint32_t rowSize);
public:
    /**
     * Spans a temporally extended network of size <numberOfTimepoints> X <numberOfFluents>
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

    static Gecode::Int::IntView getView(Gecode::ViewArray<Gecode::Int::IntView>& view,
            uint32_t col, uint32_t row,
            uint32_t numberOfFluent, uint32_t numberOfTimepoints);

    static FluentTimeIdx getFluentTimeIdx(uint32_t rowOrCol, uint32_t numberOfFluents);
};

void isPath(Gecode::Space& home, const Gecode::IntVarArgs&, uint32_t numberOfTimepoints, uint32_t numberOfFluents);

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_PROPAGATORS_IS_PATH_HPP
