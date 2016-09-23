#include "IsPath.hpp"
#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>
#include <base-logging/Logging.hpp>

#include "../utils/FluentTimeIndex.hpp"

using namespace Gecode;
using namespace templ::solvers::csp::utils;

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

Gecode::LinIntExpr IsPath::sumOfArray(const Gecode::ViewArray<Int::IntView>& view, uint32_t from, uint32_t n)
{
    Gecode::LinIntExpr expr = 0;
    if(n == 0)
    {
        n = view.size() - from;
    }

    for(uint32_t offset = 0; offset < n; ++offset)
    {
        if( ((uint32_t) view.size()) <= (from + offset))
        {
            LOG_WARN_S << "IntView size is: " << view.size() << ", but access to " << from + offset << std::endl
                << "offset is " << n << "start is " << from << std::endl;
        }
        assert(view.size() > from + offset);
        expr = expr + view[from + offset];
    }
    return expr;
}

Gecode::LinIntExpr IsPath::sumOfMatrixSlice(const Gecode::ViewArray<Gecode::Int::IntView>& view, uint32_t fromCol, uint32_t fromRow, uint32_t toCol, uint32_t toRow, uint32_t rowSize)
{
    Gecode::LinIntExpr expr = 0;

    assert(fromRow <= rowSize);
    assert(toRow <= rowSize);
    assert(toCol <= rowSize);
    assert(fromCol <= rowSize);

    for(uint32_t row = fromRow; row <= toRow; ++row)
    {
        uint32_t rowBaseIdx = row*rowSize;
        for(uint32_t idx = rowBaseIdx + fromCol; idx <= rowBaseIdx + toCol; ++idx)
        {
            expr = expr + view[idx];
        }
    }
    return expr;
}

void isPath(Gecode::Space& home, const Gecode::IntVarArgs& x, uint32_t numberOfTimepoints, uint32_t numberOfFluents)
{
    // If there is no path -- fail directly
    if(x.size() == 0)
    {
        home.fail();
    } else {
        ViewArray<Int::IntView> xv(home, x);
        LOG_DEBUG_S << "IsPath: progpagate #timepoints " << numberOfTimepoints << ", #fluents " << numberOfFluents;
        if(IsPath::post(home, xv, numberOfTimepoints, numberOfFluents) != ES_OK)
        {
            home.fail();
        }
    }

    if(home.failed())
    {
        return;
    }
}

IsPath::IsPath(Gecode::Space& home, ViewArray<Int::IntView>& xv, uint32_t numberOfTimepoints, uint32_t numberOfFluents)
    : NaryPropagator<Int::IntView, Int::PC_INT_BND>(home, xv)
    , mNumberOfTimepoints(numberOfTimepoints)
    , mNumberOfFluents(numberOfFluents)
    , mNumberOfVertices(numberOfTimepoints*numberOfFluents)
    , mGraph(xv)
{
}

IsPath::IsPath(Gecode::Space& home, bool share, IsPath& p)
    : NaryPropagator<Int::IntView, Int::PC_INT_BND>(home, share, p)
{
    mGraph.update(home, share, p.mGraph);
}

Gecode::ExecStatus IsPath::post(Gecode::Space& home, ViewArray<Int::IntView>& xv, uint32_t numberOfTimepoints, uint32_t numberOfFluents)
{
    Gecode::IntConLevel intConLevel = Gecode::ICL_DEF;

    uint32_t rowSize = numberOfTimepoints*numberOfFluents;
    uint32_t colSize = rowSize;

    // Record the sum of all columns -- which should be
    // constrained to one -- i.e. no parallel edges
    std::vector<Gecode::LinIntExpr> sumOfCols;
    std::vector<Gecode::LinIntExpr> sumOfRows;

    // 1. validate time property: only valid are time-forward-connections
    // compute sums of columns and rows for connections between timepoints
    // i.e. validate transitions between timepoints
    Gecode::LinIntExpr sumOfConnections = 0;
    // The length of all
    uint32_t sameTimeColumnSize = colSize*numberOfFluents;
    uint32_t sameTimeRowSize = rowSize*numberOfFluents;

    for(uint32_t row = 0; row < colSize; ++row)
    {
        FluentTimeIndex index = FluentTimeIndex::fromRowOrCol(row, numberOfFluents, numberOfTimepoints);
        uint32_t fromFluentIdx = index.getFluentIndex();
        uint32_t fromTimeIdx = index.getTimeIndex();

        // Constrained defined here: there can be maximum 1 outgoing edge from any timepoint
        // Use the fluent with index 0 which represents the start of an arbitrary timepoint
        if(fromFluentIdx == 0)
        {
            // sum all rows that correspond to one timepoint
            // row*colSize --> row times the "width of a row" (column size) is
            // the starting index
            // colSize*numberOfFluents --> all edges that are outgoing from this
            // timepoint, since there are multiple fluents (e.g. locations) that
            // are associated with a timepoint this can cover more than one row
            //
            uint32_t fromIdx = row*rowSize; // marks the start of the array index
            Gecode::LinIntExpr sumOfOutEdges = sumOfArray(xv, fromIdx, sameTimeColumnSize);
            Gecode::LinIntRel maxOneOutEdge( sumOfOutEdges <= 1);
            maxOneOutEdge.post(home, true, intConLevel);

            if(row != colSize -1)
            {
                Gecode::BoolExpr hasOutgoingEdge(sumOfOutEdges == 1);

                // Make sure that either the next timepoint has an outgoing transition or no
                // other
                uint32_t nextIdx = row + numberOfFluents;
                Gecode::LinIntExpr sumOfNextOutEdgesRowWise = sumOfMatrixSlice(xv, 0, nextIdx, colSize -1, std::min(rowSize, nextIdx + numberOfFluents)-1, rowSize);
                Gecode::LinIntExpr sumOfNextOutEdgesColumnWise = sumOfMatrixSlice(xv, nextIdx,0, std::min(rowSize, nextIdx + numberOfFluents) -1, rowSize -1, rowSize);

                Gecode::LinIntRel minOneOutEdge(sumOfNextOutEdgesRowWise + sumOfNextOutEdgesColumnWise == 2);

                Gecode::LinIntExpr sumOfRemainingEdges = sumOfMatrixSlice(xv, 0, nextIdx, colSize-1, rowSize -1, rowSize);
                Gecode::LinIntRel noOtherOutEdge(sumOfRemainingEdges == 0);

                Gecode::BoolExpr endPoint(minOneOutEdge, BoolExpr::NT_OR, noOtherOutEdge);

                Gecode::BoolExpr connectionRequirement(!hasOutgoingEdge, BoolExpr::NT_OR, endPoint);
                Gecode::BoolVar isValidEdge = connectionRequirement.expr(home, intConLevel);

                Gecode::LinIntRel valid(isValidEdge, Gecode::IRT_EQ, 1);
                valid.post(home, true, intConLevel);
            }
        }

        Gecode::LinIntExpr sumOfRow = 0;
        // Constrained defined here: edges can be only link two vertices forward in time
        //############################
        //    forward in time only
        //###########################
        //        t0-l0 ... t0-l2   t1-l1 ...
        // t0-l0    x        x       ok
        // t0-l1    x        x       ok
        // t0-l2    x        x       ok
        // t1-l1    x        x       x        x
        // t1-l2    x        x       x        x
        //
        for(uint32_t col = 0; col < rowSize; ++col)
        {
            FluentTimeIndex toIdx = FluentTimeIndex::fromRowOrCol(col, numberOfFluents, numberOfTimepoints);
            uint32_t toFluentIdx = toIdx.getFluentIndex();
            uint32_t toTimeIdx = toIdx.getTimeIndex();

            Gecode::Int::IntView sourceElementView = getView(xv, col, row, numberOfFluents, numberOfTimepoints);

            // Constrain the transition to go forward in time only
            // 0: no edge, i.e. transition not allowed
            if(toTimeIdx <= fromTimeIdx)
            {
                Gecode::LinIntExpr expr = 0 + sourceElementView;
                Gecode::LinIntRel r(expr,Gecode::IRT_EQ,0);
                r.post(home, true, intConLevel);
            }


            {
                Gecode::BoolExpr hasOutgoingEdge(sourceElementView == 1);

                // Make sure that either the target row which is defined by the column index has an outgoing edge or
                // no other
                uint32_t nextRowIdx = col;
                uint32_t nextColStart = (col - toFluentIdx) + numberOfFluents;
                uint32_t nextColEnd = nextColStart + numberOfFluents - 1;
                if(nextColEnd < colSize)
                {
                    Gecode::LinIntExpr sumOfNextOutEdgesTargetRow = sumOfMatrixSlice(xv, nextColStart, nextRowIdx, nextColEnd, nextRowIdx, rowSize);
                    Gecode::LinIntRel minOneOutEdge(sumOfNextOutEdgesTargetRow == 1);

                    Gecode::LinIntExpr sumOfRemainingEdges = sumOfMatrixSlice(xv, 0, nextRowIdx-toFluentIdx, colSize-1, rowSize -1, rowSize);
                    Gecode::LinIntRel noOtherOutEdge(sumOfRemainingEdges == 0);

                    Gecode::BoolExpr endPoint(minOneOutEdge, BoolExpr::NT_OR, noOtherOutEdge);

                    Gecode::BoolExpr connectionRequirement(!hasOutgoingEdge, BoolExpr::NT_OR, endPoint);
                    Gecode::BoolVar isValidEdge = connectionRequirement.expr(home, intConLevel);

                    Gecode::LinIntRel valid(isValidEdge, Gecode::IRT_EQ, 1);
                    valid.post(home, true, intConLevel);
                }
            }

            // Compute the sum of all connections
            sumOfConnections = sumOfConnections + sourceElementView;
            if(sumOfCols.size() <= col)
            {
                Gecode::LinIntExpr sum = 0;
                sumOfCols.push_back(sum);
            }
            sumOfCols[col] = sumOfCols[col] + sourceElementView;

            sumOfRow = sumOfRow + sourceElementView;
        }
        // Gather the sums of rows
        sumOfRows.push_back(sumOfRow);
    }

    // Make sure there is at least one connection -- otherwise this is
    // not a path
    Gecode::LinIntRel r(sumOfConnections >= 1);
    r.post(home, true, intConLevel);

    // documentation. 4.3.1 Post functions are clever
    // A constraint post function carefully analyzes its arguments. Based on
    // this analysis, the constraint post function chooses the best possible propagator for the constraint
    (void) new (home) IsPath(home, xv, numberOfTimepoints, numberOfFluents);
    return ES_OK;
}

size_t IsPath::dispose(Gecode::Space& home)
{
    mGraph.cancel(home, *this, Int::PC_INT_DOM);
    (void) Propagator::dispose(home);
    return sizeof(*this);
}

Gecode::Propagator* IsPath::copy(Gecode::Space& home, bool share)
{
    return new (home) IsPath(home, share, *this);
}

Gecode::PropCost IsPath::cost(const Gecode::Space&, const Gecode::ModEventDelta&) const
{
    return Gecode::PropCost::quadratic(PropCost::LO, mGraph.size());
}

Gecode::ExecStatus IsPath::propagate(Gecode::Space& home, const Gecode::ModEventDelta&)
{
    if(mGraph.assigned())
    {
        return home.ES_SUBSUMED(*this);
    } else {
        // the propagator will be scheduled if one of its views have been modified
        return ES_NOFIX;
    }
}

Int::IntView IsPath::getView(Gecode::ViewArray<Gecode::Int::IntView>& view, uint32_t col, uint32_t row,
        uint32_t numberOfFluents,
        uint32_t numberOfTimepoints)
{
    uint32_t sizeOfCol = numberOfFluents*numberOfTimepoints;
    uint32_t arrayIdx = row*sizeOfCol + col;

    assert(arrayIdx < view.size());
    return view[arrayIdx];
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

