#include "IsPath.hpp"
#include <algorithm>
#include <gecode/set.hh>
#include <gecode/set/rel.hh>
#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>
#include <base-logging/Logging.hpp>
#include <sstream>

#include "../utils/FluentTimeIndex.hpp"

using namespace Gecode;
using namespace templ::solvers::csp::utils;

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

// Advisor for the isPath propagator

// std::string IsPath::Idx::toString() const
// {
//     std::stringstream ss;
//     ss << "Idx: " << idx() << " tp: "<< isTimepointIdx();
//     return ss.str();
// }
//
// IsPath::Idx::Idx(Gecode::Space& home, Gecode::Propagator& p,
//         Gecode::Council<Idx>& c, int i, bool isTimepointIdx, SetVarArrayView x)
//     : Gecode::Advisor(home, p, c)
//     , mInfo(i << 1)
//     , mIsTimepointIdx(isTimepointIdx)
//     , x(x)
// {
//     // Subscribe to view
//     x.subscribe(home, *this);
// }
//
// IsPath::Idx::Idx(Gecode::Space& home, bool share, Idx& other)
//     : Gecode::Advisor(home, share, other)
//     , mInfo(other.mInfo)
//     , mIsTimepointIdx(other.mIsTimepointIdx)
// {
//     x.update(home, share, other.x);
// }
//
// void IsPath::Idx::dispose(Gecode::Space& home, Gecode::Council<Idx>& c)
// {
//     LOG_WARN_S << "ADVISOR IS BEING DISPOSED: " << toString();
//     x.cancel(home, *this);
//     Advisor::dispose(home, c);
// }



void isPath(Gecode::Space& home, const Gecode::SetVarArgs& x, uint32_t numberOfTimepoints, uint32_t numberOfFluents, int minPathLength, int maxPathLength)
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

IsPath::IsPath(Gecode::Space& home, ViewArray<Set::SetView>& xv, uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        int minPathLength,
        int maxPathLength)

    : NaryPropagator<Set::SetView, Set::PC_SET_VAL>(home, xv)
    , mNumberOfTimepoints(numberOfTimepoints)
    , mNumberOfFluents(numberOfFluents)
    , mMinPathLength(minPathLength)
    , mMaxPathLength(maxPathLength)
    , mAssignedTimepoints(numberOfTimepoints, std::pair<int, bool>(-1, false) )
{
    // If the home space is deleted this makes sure that the propagator will
    // also be deleted
    // see http://www.gecode.org/doc-latest/reference/group__TaskActor.html
    home.notice(*this, Gecode::AP_DISPOSE);

    assert(mNumberOfTimepoints != 0);
    assert(mNumberOfFluents != 0);

    //// Create location idx
    //// to group all space-time-point that refer to the same location
    //// in one group for observation
    //std::vector< SetVarArrayView > locationIdxList(mNumberOfFluents);

    //for(size_t l = 0; l < mNumberOfFluents; ++l)
    //{
    //    locationIdxList[l] = SetVarArrayView(home, mNumberOfTimepoints);
    //}

    //// Group all space-time-point that refer to the same timepoint
    //// in one group for observation
    //for(size_t t = mNumberOfTimepoints; t--; )
    //{
    //    SetVarArrayView arrayView(home, mNumberOfFluents);

    //    size_t offset = t*mNumberOfFluents;
    //    for(size_t l = mNumberOfFluents; l--; )
    //    {
    //        size_t idx = offset + l;

    //        arrayView[l] = x[idx];
    //        locationIdxList[l][t] = x[idx];
    //    }
    //    // Create array view for timepoints
    //    (void) new (home) Idx(home, *this, c, t, true, arrayView);
    //}

    //for(size_t l = 0; l < mNumberOfFluents; ++l)
    //{
    //    (void) new (home) Idx(home, *this, c, l, false, locationIdxList[l]);
    //}

}

IsPath::IsPath(Gecode::Space& home, bool share, IsPath& p)
    : NaryPropagator<Set::SetView, Set::PC_SET_VAL>(home, share, p)
    , mNumberOfTimepoints(p.mNumberOfTimepoints)
    , mNumberOfFluents(p.mNumberOfFluents)
    , mMinPathLength(p.mMinPathLength)
    , mMaxPathLength(p.mMaxPathLength)
    , mAssignedTimepoints(p.mAssignedTimepoints)
{
    x.update(home, share, p.x);
}

Gecode::ModEvent IsPath::disableSametimeView(Gecode::Space& home, int viewIdx)
{
    size_t fluentTimePoint = viewIdx;
    size_t fluentIdx = fluentTimePoint % mNumberOfFluents;
    size_t startIdx = fluentTimePoint - fluentIdx;
    for(size_t idx = startIdx; idx < startIdx + mNumberOfFluents; ++idx)
    {
        if(idx != fluentTimePoint)
        {
            Gecode::Set::SetView& v = x[idx];
            // disable concurrent nodes to a target node
            GECODE_ME_CHECK( v.cardMax(home, 0) );
        }
    }
    return Gecode::ME_GEN_ASSIGNED;
}

Gecode::ModEvent IsPath::constrainSametimeView(Gecode::Space& home, int viewIdx, int lowerBound, int upperBound)
{
    int fluentTimePoint = viewIdx;
    int fluentIdx = fluentTimePoint % mNumberOfFluents;
    int startIdx = fluentTimePoint - fluentIdx;
    size_t endIdx = startIdx + mNumberOfFluents;
    bool failed = false;
    for(size_t idx = startIdx; idx < endIdx; ++idx)
    {
        Gecode::Set::SetView& v = x[idx];
        GECODE_ME_CHECK( v.intersect(home, lowerBound, upperBound) );
    }
    return Gecode::ME_GEN_ASSIGNED;
}

bool IsPath::isValidWaypointSequence(const std::vector< std::pair<int,bool> >& waypoints,
        size_t& start,
        size_t& end)
{
    start = 0;
    end = 0;

    if(waypoints.size() <= 1)
    {
        return true;
    }

    bool foundStart = false;
    bool finalized = false;
    size_t validWaypoints = 0;

    for(size_t i = 0; i < waypoints.size(); ++i)
    {
        const std::pair<int, bool>& current = waypoints[i];
        // Initialize start when full assignment of a row as indicated by the
        // second argument is given
        if(current.second) // row is fully initialized
        {
            if(current.first != -1) // and there is a given waypoint
            {
                if(finalized) // path is finalized so there should be no more waypoint
                {
                    return false;
                }

                if(!foundStart) // if start is not yet initialized
                {
                    foundStart = true;
                    start = i;
                }

                end = i;
                ++validWaypoints;
            } else { // there is no given waypoint
                if(foundStart)
                {
                    if(!finalized) // finalize when encountering an empty row
                    {
                        finalized = true;
                    }
                }
            }
        } else { // row is not fully initialized
            if(!finalized)
            {
                // Count not yet assigned rows as valid waypoints
                ++validWaypoints;
                end = i;
            }
        }

        if(foundStart)
        {
            // Compute known pathlength
            size_t pathlength = end - start;
            if(pathlength != validWaypoints - 1)
            {
                return false;
            }
        }
    }
    return true;
}

Gecode::ExecStatus IsPath::post(Gecode::Space& home, const Gecode::SetVarArgs& x, uint32_t numberOfTimepoints, uint32_t numberOfFluents, int minPathLength, int maxPathLength)
{
    Gecode::IntVarArray allCardinalities(home, numberOfTimepoints*numberOfFluents, 0, 1);

    // Setup the basic contraints for the timeline
    // i.e. only edges from one timestep to the next are allowed
    for(size_t t = 0; t < numberOfTimepoints; ++t)
    {
        Gecode::IntVarArray cardinalities(home, numberOfFluents, 0, 1);
        for(size_t l = 0; l < numberOfFluents; ++l)
        {
            int idx = t*numberOfFluents + l;
            const Gecode::SetVar& edgeActivation = x[idx];
            // Use SetView to manipulate the edgeActivation in the
            // timeline
            Gecode::Set::SetView v(edgeActivation);
            // http://www.gecode.org/doc-latest/reference/classGecode_1_1Set_1_1SetView.html
            // set value to 'col' which represents the next target
            // space-time-point
            v.cardMin(home, 0);
            v.cardMax(home, 1);
            // exclude space-time-points outside the next step
            v.exclude(home, 0, (t+1)*numberOfFluents - 1);
            v.exclude(home, (t+2)*numberOfFluents, numberOfTimepoints*numberOfFluents);

            Gecode::cardinality(home, edgeActivation, cardinalities[l]);
            Gecode::cardinality(home, edgeActivation, allCardinalities[idx]);
        }

        // Limit to one outgoing edge per timestep
        // Less or equal cardinality of 1
        // sum of cardinalities
        Gecode::linear(home, cardinalities, Gecode::IRT_LQ, 1);
    }

    // Constraint the path length
    Gecode::linear(home, allCardinalities, Gecode::IRT_GQ, minPathLength);
    // Constraint the path length only if it makes sense
    if(maxPathLength < static_cast<int>(numberOfTimepoints*numberOfFluents) )
    {
        Gecode::linear(home, allCardinalities, Gecode::IRT_LQ, maxPathLength);
    }

    // documentation. 4.3.1 Post functions are clever
    // A constraint post function carefully analyzes its arguments. Based on
    // this analysis, the constraint post function chooses the best possible propagator for the constraint
    ViewArray<Set::SetView> viewArray(home, x);
    (void) new (home) IsPath(home, viewArray, numberOfTimepoints, numberOfFluents, minPathLength, maxPathLength);
    return ES_OK;
}

size_t IsPath::dispose(Gecode::Space& home)
{
    home.ignore(*this, AP_DISPOSE);
    (void) NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_VAL>::dispose(home);
    return sizeof(*this);
}

Gecode::Propagator* IsPath::copy(Gecode::Space& home, bool share)
{
    return new (home) IsPath(home, share, *this);
}

Gecode::PropCost IsPath::cost(const Gecode::Space&, const Gecode::ModEventDelta&) const
{
    return Gecode::PropCost::quadratic(PropCost::LO, x.size());
}

void IsPath::reschedule(Gecode::Space& home)
{
    Gecode::Set::SetView::schedule(home, *this, Gecode::ME_GEN_ASSIGNED);
}

//Gecode::ExecStatus IsPath::advise(Gecode::Space& home, Gecode::Advisor& a, const Gecode::Delta& d)
//{
//    ModEvent me = Gecode::Set::SetView::modevent(d);
//    if(me == Gecode::ME_GEN_ASSIGNED)
//    {
//        Idx& advisor = static_cast<Idx&>(a);
//
//        int idx = advisor.idx();
//        if(advisor.isTimepointIdx())
//        {
//            mAssignedTimepointIndices.insert( idx );
//        } else {
//            mAssignedFluentIndices.insert( idx );
//        }
//        return ES_NOFIX;
//    }
//    // Do not schedule unless a view has been assigned
//    return ES_FIX;
//}

Gecode::ExecStatus IsPath::propagate(Gecode::Space& home, const Gecode::ModEventDelta&)
{
    for(size_t timepoint = 0; timepoint < mNumberOfTimepoints; ++timepoint)
    {
        // reset full assignment
        bool fullyAssigned = true;
        int currentWaypoint = -1;

        // While we could use advisors as well, this seems to be for a first
        // approach simpler -- since we hit some seg fault when iterating the
        // council
        // Due to the implemented propagation -- when a single value is assigned
        // then all values of the same timepoint will be properly assigned as
        // well -- so we can safely skip any further propagation here
        // Skip timepointrow that have already been handled
        if( mAssignedTimepoints[timepoint].second)
        {
            LOG_DEBUG_S << "Skipping already assigned waypoint: " << timepoint << " " << mAssignedTimepoints[timepoint].first;
            continue;
        }

        int offset = timepoint*mNumberOfFluents;
        for(size_t fluent = 0; fluent < mNumberOfFluents; ++fluent)
        {
            int i = offset + fluent;

            // This node has an outgoing edge
            if(x[i].assigned())
            {
                fullyAssigned |= true;

                if(x[i].lubSize() == 1)
                {
                    int prev = i - mNumberOfFluents;
                    if(prev > 0)
                    {
                        // Constrain all column of source nodes that lead to the current
                        // node i, to the domain of the current node
                        ModEvent ev = constrainSametimeView(home, prev, i, i);
                        if(ev == Gecode::ME_GEN_FAILED)
                        {
                            return ES_FAILED;
                        }
                    }
                    // There can be only one --
                    // which should not be needed since wit have established the
                    // linear constraint on cardinality already
                    ModEvent ev = disableSametimeView(home, i);
                    if(ev == Gecode::ME_GEN_FAILED)
                    {
                        return ES_FAILED;
                    }

                    ev = disableSametimeView(home, x[i].lubMax());
                    if(ev == Gecode::ME_GEN_FAILED)
                    {
                        return ES_FAILED;
                    }

                    currentWaypoint = i;
                    // Since the target is bound as next node, we
                    // can already update this status
                    mAssignedTimepoints[timepoint + 1] = std::pair<int, bool>( x[i].lubMax(), true);
                }
            } else {
                fullyAssigned = false;
            }
        }

        if(fullyAssigned)
        {
            mAssignedTimepoints[timepoint] = std::pair<int, bool>( currentWaypoint, fullyAssigned );
        }
    }

    size_t start;
    size_t end;
    if(!isValidWaypointSequence(mAssignedTimepoints, start, end))
    {
        return ES_FAILED;
    }

    if(x.assigned())
    {
        return home.ES_SUBSUMED(*this);
    } else {
        // the propagator will be scheduled if one of its views have been modified
        return ES_FIX;
    }
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

