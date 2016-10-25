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
        ViewArray<Set::SetView> xv(home, x);

        LOG_DEBUG_S << "IsPath: propagate #timepoints " << numberOfTimepoints << ", #fluents " << numberOfFluents;
        if(IsPath::post(home, xv, numberOfTimepoints, numberOfFluents, minPathLength, maxPathLength) != ES_OK)
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

Gecode::ExecStatus IsPath::post(Gecode::Space& home, ViewArray<Set::SetView>& xv, uint32_t numberOfTimepoints, uint32_t numberOfFluents, uint32_t minPathLength, uint32_t maxPathLength)
{
    int32_t numberOfVertices = numberOfFluents*numberOfTimepoints;
    // 1. validate time property: only valid are time-forward-connections
    // compute sums of columns and rows for connections between timepoints
    // i.e. validate transitions between timepoints
    SetVarArray all(home, numberOfVertices);
    for(size_t timeIdx = 0; timeIdx < numberOfTimepoints; ++timeIdx)
    {
        SetVarArray sameTimeViews(home, numberOfFluents);
        IntVarArray cardinalities(home, numberOfFluents, 0, 1);
        for(size_t fluentIdx = 0; fluentIdx < numberOfFluents; ++fluentIdx)
        {
            // Allowed target nodes are forward in time
            int from = (timeIdx+1)*numberOfFluents;
            int to = from + numberOfFluents;
            SetVar allowedTargets;
            if(timeIdx != numberOfTimepoints -1)
            {
                allowedTargets = SetVar(home, Gecode::IntSet::empty, Gecode::IntSet(from,to),0,1);
            } else {
                allowedTargets = SetVar(home, Gecode::IntSet::empty, Gecode::IntSet::empty,0,0);
            }

            LOG_DEBUG_S << "FROM: " << from << ", TO: " << to << "  timeIdx: " << timeIdx << ", mNumberOfFluents: " << numberOfFluents;
            LOG_DEBUG_S << (timeIdx*numberOfFluents + fluentIdx) << " -- ALLOWED TARGET: " << allowedTargets;

            int currentIdx = timeIdx*numberOfFluents + fluentIdx;

            assert(currentIdx < numberOfVertices);

            Gecode::SetVar currentView(xv[currentIdx]);
            Gecode::rel(home, currentView <= allowedTargets);

            sameTimeViews[fluentIdx] = currentView;
            Gecode::cardinality(home, currentView, cardinalities[fluentIdx]);

            all[timeIdx*numberOfFluents+fluentIdx] = currentView;
        }

        // 2. There should be at maximum one outgoing edge (for parallel time)
        rel(home, sum(cardinalities) <= 1);
    }

    Gecode::SetVar allUnion(home);
    Gecode::rel(home, SOT_UNION, all, allUnion);

    // 3. There should be a minimum of one edge
    Gecode::IntVar allUnionCardinality(home, 0, numberOfVertices);
    Gecode::cardinality(home, allUnion, allUnionCardinality);
    Gecode::rel(home, allUnionCardinality >= std::min(numberOfTimepoints - 1, minPathLength) );
    Gecode::rel(home, allUnionCardinality <= std::min(numberOfTimepoints - 1, maxPathLength) );

    // 4. The path should be without holes
    Gecode::IntVar minAll(home, 0, numberOfVertices);
    Gecode::min(home, allUnion, minAll);

    Gecode::IntVar maxAll(home, 0, numberOfVertices);
    Gecode::min(home, allUnion, maxAll);

    // Identification of the timepoints for the particular egde
    //
    // (tp*|F| + f) - (tp*|F| + f)%(|F|) = tp*|F|
    Gecode::rel(home, ((maxAll - (maxAll%numberOfFluents)) - (minAll-(minAll%numberOfFluents))) <= ((allUnionCardinality-1)*numberOfFluents));

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

