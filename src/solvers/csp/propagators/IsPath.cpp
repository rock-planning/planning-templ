#include "IsPath.hpp"
#include <gecode/set.hh>
#include <gecode/set/rel.hh>
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

void isPath(Gecode::Space& home, const Gecode::SetVarArgs& x, uint32_t numberOfTimepoints, uint32_t numberOfFluents, uint32_t minPathLength, uint32_t maxPathLength)
{
    // If there is no path -- fail directly
    if(x.size() == 0)
    {
        home.fail();
    } else {

        LOG_DEBUG_S << "IsPath: propagate #timepoints " << numberOfTimepoints << ", #fluents " << numberOfFluents;
        if(IsPath::post(home, x, numberOfTimepoints, numberOfFluents, minPathLength, maxPathLength) != ES_OK)
        {
            home.fail();
        }
    }

    if(home.failed())
    {
        return;
    }
}

IsPath::IsPath(Gecode::Space& home, ViewArray<Set::SetView>& xv, uint32_t numberOfTimepoints, uint32_t numberOfFluents)
    : NaryPropagator<Set::SetView, Set::PC_SET_ANY>(home, xv)
    , mNumberOfTimepoints(numberOfTimepoints)
    , mNumberOfFluents(numberOfFluents)
    , mNumberOfVertices(numberOfTimepoints*numberOfFluents)
    , mGraph(xv)
{
}

IsPath::IsPath(Gecode::Space& home, bool share, IsPath& p)
    : NaryPropagator<Set::SetView, Set::PC_SET_ANY>(home, share, p)
{
    mGraph.update(home, share, p.mGraph);
}

void IsPath::disableSametimeView(Gecode::Space& home, const Gecode::SetVarArgs& x, int viewIdx, uint32_t numberOfTimepoints, uint32_t numberOfFluents)
{
    int fluentTimePoint = viewIdx;
    int fluentIdx = fluentTimePoint % numberOfFluents;
    int startIdx = fluentTimePoint - fluentIdx;
    for(int idx = startIdx; idx < startIdx + numberOfFluents; ++idx)
    {
        if(idx != fluentTimePoint)
        {
            Gecode::Set::SetView v(x[idx]);
            // disable concurrent nodes to a target node
            v.cardMax(home, 0);
        }
    }
}

Gecode::ExecStatus IsPath::post(Gecode::Space& home, const Gecode::SetVarArgs& x, uint32_t numberOfTimepoints, uint32_t numberOfFluents, uint32_t minPathLength, uint32_t maxPathLength)
{
    for(int i = 0; i < x.size(); ++i)
    {
        if(x[i].assigned() && x[i].lubSize() == 1)
        {
            disableSametimeView(home, x, i, numberOfTimepoints, numberOfFluents);
            disableSametimeView(home, x, x[i].lubMax(), numberOfTimepoints, numberOfFluents);
        }
    }



//    int32_t numberOfVertices = numberOfFluents*numberOfTimepoints;
//    // 1. validate time property: only valid are time-forward-connections
//    // compute sums of columns and rows for connections between timepoints
//    // i.e. validate transitions between timepoints
//    SetVarArray all(home, numberOfVertices);
//    SetVar path(home, Gecode::IntSet::empty, Gecode::IntSet(0,numberOfVertices),1,numberOfTimepoints);
//    Set::SetView viewPath(path);
//
//    for(size_t timeIdx = 0; timeIdx < numberOfTimepoints; ++timeIdx)
//    {
//        // This array contains views, that all have the same associated timepoint
//        SetVarArray sameTimeViews(home, numberOfFluents);
//        // This array contains the cardinalites of the 'sameTimeViews'
//        IntVarArray cardinalities(home, numberOfFluents, 0, 1);
//        for(size_t fluentIdx = 0; fluentIdx < numberOfFluents; ++fluentIdx)
//        {
//            // Allowed target nodes are forward in time
//            // otherwise they are constrained to the empty set
//            int lowerBoundTo = (timeIdx+1)*numberOfFluents;
//            int upperBoundTo = lowerBoundTo + numberOfFluents;
//            SetVar allowedTargets;
//            if(timeIdx != numberOfTimepoints -1)
//            {
//                allowedTargets = SetVar(home, Gecode::IntSet::empty, Gecode::IntSet(lowerBoundTo,upperBoundTo),0,1);
//            } else {
//                allowedTargets = SetVar(home, Gecode::IntSet::empty, Gecode::IntSet::empty,0,0);
//            }
//
//            LOG_DEBUG_S << "Lower bound to: " << lowerBoundTo << ", upper bound to: " << upperBoundTo << "  timeIdx: " << timeIdx << ", mNumberOfFluents: " << numberOfFluents;
//            LOG_DEBUG_S << (timeIdx*numberOfFluents + fluentIdx) << " -- ALLOWED TARGET: " << allowedTargets;
//
//            int currentIdx = timeIdx*numberOfFluents + fluentIdx;
//            assert(currentIdx < numberOfVertices);
//
//            Gecode::SetVar currentView(x[currentIdx]);
//            Gecode::rel(home, currentView <= allowedTargets);
//
//            // Cache or channel values
//            sameTimeViews[fluentIdx] = currentView;
//            Gecode::cardinality(home, currentView, cardinalities[fluentIdx]);
//            all[timeIdx*numberOfFluents+fluentIdx] = currentView;
//        }
//
//        // 2. There should be at maximum one outgoing edge (for parallel time)
//        rel(home, sum(cardinalities) <= 1);
//    }
//
//    Gecode::SetVar allUnion(home);
//    Gecode::rel(home, SOT_UNION, all, allUnion);
//
//    Gecode::SetVar z(home);
//    Gecode::element(home, SOT_UNION, x, allUnion, z);
//
//    Gecode::IntVar expectedTargetsCardinality(home, 0, numberOfTimepoints);
//    Gecode::cardinality(home, z, expectedTargetsCardinality);
//
//    // 3. There should be a minimum of one edge, i.e. the union across all sets
//    // should have a cardinality that corresponds to the pathlength
//    Gecode::IntVar allUnionCardinality(home, 0, numberOfTimepoints);
//    Gecode::cardinality(home, allUnion, allUnionCardinality);
//    Gecode::rel(home, allUnionCardinality >= std::min(numberOfTimepoints - 1, minPathLength) );
//    Gecode::rel(home, allUnionCardinality <= std::min(numberOfTimepoints - 1, maxPathLength) );
//
//    // 3.a The path should be consistent, from-0 --> to-0, from-1 --> to-1
//    // , where to-0 == from-1
//    //Gecode::IntVar pathCardinality(home, 0, numberOfTimepoints);
//    //Gecode::cardinality(home, path, pathCardinality);
//    //Gecode::rel(home, allUnionCardinality == (pathCardinality + 1));
//    Gecode::rel(home, allUnionCardinality <= (expectedTargetsCardinality+1));
//
//    // 4. The path should be without holes
//    Gecode::IntVar minAll(home, 0, numberOfVertices);
//    Gecode::min(home, allUnion, minAll);
//
//    Gecode::IntVar maxAll(home, 0, numberOfVertices);
//    Gecode::min(home, allUnion, maxAll);
//
//    // Identification of the timepoints for the particular egde
//    //
//    // (tp*|F| + f) - (tp*|F| + f)%(|F|) = tp*|F|
//    Gecode::rel(home, ((maxAll - (maxAll%numberOfFluents)) - (minAll-(minAll%numberOfFluents))) <= ((allUnionCardinality-1)*numberOfFluents));

    // documentation. 4.3.1 Post functions are clever
    // A constraint post function carefully analyzes its arguments. Based on
    // this analysis, the constraint post function chooses the best possible propagator for the constraint
    ViewArray<Set::SetView> viewArray(home, x);
    (void) new (home) IsPath(home, viewArray, numberOfTimepoints, numberOfFluents);
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

