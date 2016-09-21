#include "IsPath.hpp"
#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>
#include <base-logging/Logging.hpp>

using namespace Gecode;

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
    uint32_t colSize = numberOfTimepoints*numberOfFluents;

    // Linear Integer Expressions
    std::vector<Gecode::LinIntExpr> cols;
    std::vector<Gecode::LinIntExpr> rows;

    // Initialize all row and column expression variables to 0
    for(uint32_t i = 0; i < rowSize; ++i)
    {
        Gecode::LinIntExpr expr = 0;
        cols.push_back(expr);
        rows.push_back(expr);
    }

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
    for(uint32_t row = 0; row < colSize; ++row)
    {
        uint32_t fromFluentIdx = row%numberOfFluents;
        uint32_t fromTimeIdx = (row - fromFluentIdx)/numberOfFluents;

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
            uint32_t fromIdx = row*colSize; // marks the start of the array index
            Gecode::LinIntExpr sumOfOutEdges = sumOfArray(xv, fromIdx, sameTimeColumnSize);
            Gecode::LinIntRel maxOneOutEdge( sumOfOutEdges <= 1);
            maxOneOutEdge.post(home, true, intConLevel);

            if(row != colSize -1)
            {
                Gecode::BoolExpr hasOutgoingEdge(sumOfOutEdges == 1);

                // Make sure that either the next timepoint has an outgoing transition or no
                // other
                uint32_t nextIdx = row + numberOfFluents;
                Gecode::LinIntExpr sumOfNextOutEdgesRowWise = sumOfMatrixSlice(xv, 0, nextIdx, colSize -1, nextIdx + numberOfFluents-1, rowSize);
                Gecode::LinIntExpr sumOfNextOutEdgesColumnWise = sumOfMatrixSlice(xv, nextIdx,0, nextIdx + numberOfFluents -1, rowSize -1, rowSize);

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
            FluentTimeIdx toIdx = getFluentTimeIdx(col, numberOfFluents);
            //uint32_t toFluentIdx = toIdx.first;
            uint32_t toTimeIdx = toIdx.second;

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
                Gecode::BoolExpr hasConnection(sourceElementView == 1);
                Gecode::LinIntExpr competingEdges = sumOfMatrixSlice(xv, 0, row+1, col, rowSize - 1, rowSize);
                Gecode::LinIntRel noConflictingEdges(competingEdges, Gecode::IRT_EQ, 0);

                Gecode::BoolExpr noConflict(!hasConnection, Gecode::BoolExpr::NT_OR, noConflictingEdges);
                BoolVar conflictState = noConflict.expr(home, intConLevel);
                Gecode::LinIntRel rel(conflictState, Gecode::IRT_EQ, 1);
                rel.post(home, true, intConLevel);
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

    // Disallow parallel edges for the same target timepoint columns
    for(uint32_t col = 0; col < colSize; ++col)
    {
        uint32_t toFluentIdx = col%numberOfFluents;
        uint32_t toTimeIdx = (col-toFluentIdx)/numberOfFluents;

        if(col < colSize -1)
        {
            Gecode::LinIntRel rel(sumOfRows[col] == sumOfCols[col+1]);
            rel.post(home, true, intConLevel);
        }

        //if(toFluentIdx == 0)
        //{
        //    Gecode::LinIntExpr sumOfSameTimeCols = 0;
        //    for(size_t i = 0; i < numberOfFluents; ++i)
        //    {
        //        sumOfSameTimeCols = sumOfSameTimeCols + sumOfCols[col+i];
        //    }
        //    Gecode::LinIntRel noParallelEdges(sumOfSameTimeCols, Gecode::IRT_LQ, 1);
        //    noParallelEdges.post(home, true, intConLevel);
        //}

        // Check if this space-time has an incoming edge
        // if so, then
        // (a) its direct target has to have either an outgoing edge or
        // no other edge can be present
        // (b) no previous timepoint locations can have an outgoing edge
        Gecode::BoolExpr hasIncomingEdge(sumOfCols[col] == 1);
        Gecode::LinIntRel hasOutgoingEdge(sumOfRows[col], Gecode::IRT_EQ,1);

        Gecode::LinIntExpr sumOfRemainingRows = 0;
        // or there is not connection at all following
        for(uint32_t remainingRowIdx = col; remainingRowIdx < colSize; ++remainingRowIdx)
        {
            sumOfRemainingRows = sumOfRemainingRows + sumOfRows[remainingRowIdx];
        }
        Gecode::LinIntRel noLaterOutgoingEdge(sumOfRemainingRows, Gecode::IRT_EQ,0);

        Gecode::BoolExpr isIncrementalPath(hasOutgoingEdge, BoolExpr::NT_OR, noLaterOutgoingEdge);
        Gecode::BoolVar var = isIncrementalPath.expr(home, intConLevel);

        Gecode::LinIntRel requireTrue(var, Gecode::IRT_EQ, 1);
        //requireTrue.post(home, true, intConLevel);
    }

    // Make sure there is at least one connection -- otherwise this is
    // not a path
    Gecode::LinIntRel r(sumOfConnections >= 1);
    r.post(home, true, intConLevel);

    // 2. validate location property: first and last location have no entry link, other have in a out property
    // otherwise check for every transition that it is a valid one

    //// Sum of column or row must be <= 1 to form a path
    //// global lower bound, local upper bound
    ////SetVar (Space &home, const IntSet &glbD, const IntSet &lubD, unsigned int cardMin=0, unsigned int cardMax=Set::Limits::card)
    //Gecode::SetVar var(home,IntSet::empty, IntSet(0,rowSize-1),0,rowSize);
    //for(uint32_t index = 0; index < rowSize; ++index)
    //{
    //    {
    //        Gecode::LinIntRel r(cols[index],Gecode::IRT_LQ,1);
    //        r.post(home,true, intConLevel);

    //        // Sum of index is one
    //        BoolVar boolVar(home,0,1);

    //        Gecode::BoolExpr l(cols[index] == 0);
    //        Gecode::dom(home, var, Gecode::SRT_SUP, index, index);

    //        Gecode::LinIntExpr pathSize = var.glbMin() - var.glbMin();
//  //          Gecode::LinIntRel r =

    //        //// Implication not path element or column is member
    //        //BoolExpr membership(BoolExpr(l,BoolExpr::NT_NOT),
    //        //    BoolExpr::NT_OR,elementRequirement);
    //        //membership.expr(home, intConLevel);


    //        //min(home, start, involvedNodes);
    //        //max(home, end, involvedNodes);
    //    }
    //    {
    //        // alternative: r = (rows[index] <= 1);
    //        Gecode::LinIntRel r(rows[index], Gecode::IRT_LQ,1);
    //        r.post(home,true, intConLevel);
    //    }
    //    //  one outgoing edge for the same time nodes only
    //    //  sum of rows <= 1
    //    if(index%numberOfFluents == 0)
    //    {
    //        Gecode::LinIntExpr expr = 0;
    //        for(size_t offset = 0; offset < numberOfFluents; ++offset)
    //        {
    //            expr = expr + rows[index + offset];
    //        }
    //        Gecode::LinIntRel r(expr, Gecode::IRT_LQ, 1);
    //        r.post(home, true, intConLevel);
    //    }
    //}

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

IsPath::FluentTimeIdx IsPath::getFluentTimeIdx(uint32_t rowOrCol, uint32_t numberOfFluents)
{
    FluentTimeIdx idx;
    idx.first = rowOrCol % numberOfFluents;
    idx.second = (rowOrCol - idx.first)/numberOfFluents;
    return idx;
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

