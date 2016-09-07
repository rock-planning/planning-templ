#include <templ/solvers/csp/propagators/IsPath.hpp>
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

    // 1. validate time property: only valid are time-forward-connections
    // compute sums of columns and rows for connections between timepoints
    // i.e. validate transitions between timepoints
    Gecode::LinIntExpr sumOfConnections = 0;
    for(uint32_t row = 0; row < colSize; ++row)
    {
        uint32_t timepointSource = row/numberOfFluents;

        // Constrained defined here: there can be maximum 1 outgoing edge from one timepoint
        // Use the row that represents the start of one timepoint
        if(row%numberOfFluents == 0)
        {
            // sum all rows that correspond to one timepoint
            // row*colSize --> row times the "width of a row" (column size) is
            // the starting index
            // colSize*numberOfFluents --> all edges that are outgoing from this
            // timepoint, since there are multiple fluents (e.g. locations) that
            // are associated with a timepoint this can cover more than one row
            //
            Gecode::LinIntExpr sumOfOutEdges = sumOfArray(xv, row*colSize, colSize*numberOfFluents);
            Gecode::LinIntRel maxOneOutEdge( sumOfOutEdges <= 1);
            maxOneOutEdge.post(home, true, intConLevel);
        }

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
            uint32_t timepointTarget = col/numberOfFluents;
            {
                assert(xv.size() > row*rowSize + col);
                Gecode::Int::IntView& elementView = xv[row*rowSize + col];
                //{
                //    Gecode::LinIntExpr& expr = rows.at(row);
                //    expr = expr + elementView;
                //}
                //{
                //    Gecode::LinIntExpr& expr = cols.at(col);
                //    expr = expr + elementView;
                //}

                // Constrain the transition to go forward in time only
                // 0: no edge, i.e. transition not allowed
                if(timepointTarget <= timepointSource)
                {
                    Gecode::LinIntExpr expr = 0 + elementView;
                    Gecode::LinIntRel r(expr,Gecode::IRT_EQ,0);
                    r.post(home, true, intConLevel);
                }

                sumOfConnections = sumOfConnections + elementView;
            }
        }

        // Check if there is a transition to the next timepoint and given
        // fluent (location), if so then make sure there is an outgoing
        // connection from this location as well (or no connection at all after that)
        if(timepointSource+1 < numberOfTimepoints)
        {
            LOG_WARN_S << "Row: " << row << " -- vertices: " << rowSize;
            for(uint32_t fluentIdx = 0; fluentIdx < numberOfFluents; ++fluentIdx)
            {
                uint32_t sourceRowIndex = timepointSource*numberOfFluents + fluentIdx;
                uint32_t targetRowIndex = (timepointSource+1)*numberOfFluents + fluentIdx;

                Gecode::Int::IntView& sourceNodeView = xv[sourceRowIndex];
                Gecode::BoolExpr hasConnection(sourceNodeView == 1);

                //std::cout << "Source row: " << sourceRowIndex << std::endl;
                //std::cout << "Target row: " << targetRowIndex << std::endl;
                //std::cout << "timepoint*fluents: " << rowSize << std::endl;
                //std::cout << "fluent idx: " << fluentIdx << std::endl;
                //std::cout << "Find target row: " << targetRowIndex << " fluents: " << numberOfFluents << std::endl;
                //std::cout << "Current view: " << sourceNodeView << std::endl;

                uint32_t startTargetOffset = (timepointSource+1)*numberOfTimepoints*numberOfFluents;
                //std::cout << "Start target offset: " << startTargetOffset << std::endl;

                // Either there is an outgoing connection
                Gecode::LinIntExpr sumOfLocalOutEdges = sumOfArray(xv,startTargetOffset, numberOfFluents);
                Gecode::LinIntRel locationRequirement(sumOfLocalOutEdges, Gecode::IRT_EQ, 1);

                // or there is not connection at all following
                Gecode::LinIntExpr sumOfRemainingOutEdges = sumOfArray(xv,startTargetOffset, rowSize*rowSize - startTargetOffset);
                Gecode::LinIntRel noLocationRequirement(sumOfRemainingOutEdges, Gecode::IRT_EQ, 0);

                BoolExpr nextPathElementRequirement(locationRequirement, BoolExpr::NT_OR, noLocationRequirement);
                BoolExpr connectionRequirement(hasConnection, BoolExpr::NT_AND, nextPathElementRequirement);
                connectionRequirement.expr(home, intConLevel);
            }
        }
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

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

