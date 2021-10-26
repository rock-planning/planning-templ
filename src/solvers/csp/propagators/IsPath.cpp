#include "IsPath.hpp"
#include <algorithm>
#include <gecode/set.hh>
#include <gecode/set/rel.hh>
#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>
#include <base-logging/Logging.hpp>
#include <sstream>
#include <boost/numeric/conversion/cast.hpp>

#include "../utils/FluentTimeIndex.hpp"
#include "../utils/Formatter.hpp"

using namespace Gecode;
using namespace templ::solvers::csp::utils;

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

void isPath(Gecode::Space& home, const Gecode::SetVarArgs& x, const std::string& tag, uint32_t numberOfTimepoints, uint32_t numberOfFluents, int minPathLength, int maxPathLength)
{
    if(maxPathLength < 0)
    {
        maxPathLength = numberOfTimepoints;
    }

    // If there is no path -- fail directly
    if(x.size() == 0)
    {
        home.fail();
    } else {

        LOG_DEBUG_S << "IsPath: propagate #timepoints " << numberOfTimepoints << ", #fluents " << numberOfFluents;
        if(IsPath::post(home, x, tag, numberOfTimepoints, numberOfFluents, minPathLength, maxPathLength) != ES_OK)
        {
            home.fail();
        }
    }

    if(home.failed())
    {
        return;
    }
}

IsPath::IsPath(Gecode::Space& home, ViewArray<Set::SetView>& xv,
        const std::string& tag,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        int minPathLength,
        int maxPathLength)

    : NaryPropagator<Set::SetView, Set::PC_SET_NONE>(home, xv)
    , c(home)
    , mTag(tag)
    , mNumberOfTimepoints(numberOfTimepoints)
    , mNumberOfFluents(numberOfFluents)
    , mMinPathLength(minPathLength)
    , mMaxPathLength(maxPathLength)
    , mAssignedTimepoints(numberOfTimepoints, std::pair<int, bool>(-1, false) )
{
    if(mMaxPathLength < 0)
    {
        mMaxPathLength = mNumberOfTimepoints;
    }
    // If the home space is deleted this makes sure that the propagator will
    // also be deleted
    // see http://www.gecode.org/doc-latest/reference/group__TaskActor.html
    home.notice(*this, Gecode::AP_DISPOSE);

    assert(mNumberOfTimepoints != 0);
    assert(mNumberOfFluents != 0);

    // IDX
    // Create location idx
    // to group all space-time-point that refer to the same location
    // in one group for observation
    std::vector< SetVarArrayView > locationIdxList(mNumberOfFluents);

    for(size_t l = 0; l < mNumberOfFluents; ++l)
    {
        locationIdxList[l] = SetVarArrayView(home, mNumberOfTimepoints);
    }

    // Group all space-time-points that refer to the same timepoint
    // in one group for observation
    for(size_t t = mNumberOfTimepoints; t--; )
    {
        SetVarArrayView arrayView(home, mNumberOfFluents);

        size_t offset = t*mNumberOfFluents;
        for(size_t l = mNumberOfFluents; l--; )
        {
            size_t idx = offset + l;

            arrayView[l] = x[idx];
            locationIdxList[l][t] = x[idx];
        }
        // Create array view for timepoints
        (void) new (home) Idx(home, *this, c, t, true, arrayView);
    }

    for(size_t l = 0; l < mNumberOfFluents; ++l)
    {
        (void) new (home) Idx(home, *this, c, l, false, locationIdxList[l]);
    }


    bool doReschedule = false;
    for(int i = 0; i < x.size(); ++i)
    {
        if(x[i].assigned())
        {
            mAssignedTimepointIndices.push_back( i/mNumberOfFluents );
            mAssignedFluentIndices.push_back( i%mNumberOfFluents );
            doReschedule = true;
        }
    }
    if(doReschedule)
    {
        reschedule(home);
    }
}

IsPath::IsPath(Gecode::Space& home, IsPath& p)
    : NaryPropagator<Set::SetView, Set::PC_SET_NONE>(home, p)
    , mTag(p.mTag)
    , mNumberOfTimepoints(p.mNumberOfTimepoints)
    , mNumberOfFluents(p.mNumberOfFluents)
    , mMinPathLength(p.mMinPathLength)
    , mMaxPathLength(p.mMaxPathLength)
    , mAssignedTimepoints(p.mAssignedTimepoints)
{
    x.update(home, p.x);
    c.update(home, p.c);
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
    for(size_t idx = startIdx; idx < endIdx; ++idx)
    {
        Gecode::Set::SetView& v = x[idx];
        GECODE_ME_CHECK( v.intersect(home, lowerBound, upperBound) );
    }
    return Gecode::ME_GEN_ASSIGNED;
}

Gecode::ModEvent IsPath::excludeTarget(Gecode::Space& home, int viewIdx, int
        targetIdx)
{
    for(size_t idx = viewIdx; idx < viewIdx + mNumberOfFluents; ++idx)
    {
        Gecode::Set::SetView& v = x[idx];
        GECODE_ME_CHECK( v.exclude(home, targetIdx) );
    }
    return Gecode::ME_GEN_ASSIGNED;
}

bool IsPath::isValidWaypointSequence(const std::vector< std::pair<int,bool> >& waypoints,
        size_t& start,
        size_t& end,
        bool fullyAssigned)
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
        if(current.second || fullyAssigned) // row is fully initialized
        {
            if(current.first != -1) // and there is a given waypoint
            {
                if(finalized) // path is finalized so there should be no more waypoint
                {
                //    LOG_WARN_S << "Path is finalized -- but there is an additional waypoint";
                    return false;
                }

                if(!foundStart) // if start is not yet initialized
                {
                    foundStart = true;
                    start = i;
                }

                //LOG_WARN_S << "Valid waypoint for: " << current.first
                //    << " end at: " << i;
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
            if(foundStart && !finalized)
            {
                // Count not yet assigned rows as valid waypoints
                //LOG_WARN_S << "Valid waypoint for: " << current.first << "since not finalized"
                //    << " end at: " << i;
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
                LOG_WARN_S << "Path: " << pathlength << " but valid waypoints" << validWaypoints;
                return false;
            }
        }
    }
    return true;
}

Gecode::ExecStatus IsPath::post(Gecode::Space& home, const Gecode::SetVarArgs& x,
        const std::string& tag,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        int minPathLength, int maxPathLength)
{
    if(maxPathLength < 0)
    {
        maxPathLength = numberOfTimepoints;
    }
    // Setup the basic constraints for the timeline
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
            //v.cardMin(home, 0);
            //v.cardMax(home, 1);
            //// exclude space-time-points outside the next step
            v.exclude(home, 0, (t+1)*numberOfFluents - 1);
            v.exclude(home, (t+2)*numberOfFluents, numberOfTimepoints*numberOfFluents);

            // Edge activation and cardinality of the set correspond
            Gecode::cardinality(home, edgeActivation, cardinalities[l]);
        }

        // The min path constraint assume a start of the path at the first
        // timepoint
        if(minPathLength > boost::numeric_cast<int>(t) )
        {
            // sum of cardinality has to be 1 for the same timepoint
            Gecode::linear(home, cardinalities, Gecode::IRT_EQ, 1);
        } else if(maxPathLength < boost::numeric_cast<int>(numberOfTimepoints) && boost::numeric_cast<int>(t) > maxPathLength)
        {// Constrain the path length only if it makes sense
            Gecode::linear(home, cardinalities, Gecode::IRT_EQ, 0);
        } else {
            // Limit to one outgoing edge per timestep
            // Less or equal cardinality of 1
            // sum of cardinalities
            Gecode::linear(home, cardinalities, Gecode::IRT_LQ, 1);
        }
    }


    // documentation. 4.3.1 Post functions are clever
    // A constraint post function carefully analyzes its arguments. Based on
    // this analysis, the constraint post function chooses the best possible propagator for the constraint
    ViewArray<Set::SetView> viewArray(home, x);
    (void) new (home) IsPath(home, viewArray, tag, numberOfTimepoints, numberOfFluents, minPathLength, maxPathLength);
    return ES_OK;
}

size_t IsPath::dispose(Gecode::Space& home)
{
    home.ignore(*this, AP_DISPOSE);
    c.dispose(home);
    (void) NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_NONE>::dispose(home);
    return sizeof(*this);
}

Gecode::Propagator* IsPath::copy(Gecode::Space& home)
{
    return new (home) IsPath(home, *this);
}

Gecode::PropCost IsPath::cost(const Gecode::Space&, const Gecode::ModEventDelta&) const
{
    return Gecode::PropCost::quadratic(PropCost::LO, x.size());
}

void IsPath::reschedule(Gecode::Space& home)
{
    Gecode::Set::SetView::schedule(home, *this, Gecode::ME_GEN_ASSIGNED);
}

Gecode::ExecStatus IsPath::advise(Gecode::Space& home, Gecode::Advisor& a, const Gecode::Delta& d)
{
    ModEvent me = Gecode::Set::SetView::modevent(d);
    if(me == Gecode::ME_GEN_ASSIGNED)
    {
        Idx& advisor = static_cast<Idx&>(a);

        int idx = advisor.idx();
        if(advisor.isTimepointIdx())
        {
            mAssignedTimepointIndices.push_back( idx );
        } else {
            mAssignedFluentIndices.push_back( idx );
        }

        if(advisor.x.assigned())
        {
            return home.ES_NOFIX_DISPOSE(c, advisor);
        } else {
            return ES_NOFIX;
        }
    }
    // Do not schedule unless a view has been assigned
    return ES_FIX;
}

Gecode::ExecStatus IsPath::propagate(Gecode::Space& home, const Gecode::ModEventDelta&)
{
    //LOG_WARN_S << "Propagate: pre status: " << x
    //    << "    waypoints: " << waypointsToString()
    //    << std::endl;

    // process assignments
    while(!mAssignedTimepointIndices.empty())
    {
        int timepoint = mAssignedTimepointIndices.back();
        int fluent = mAssignedFluentIndices.back();
        mAssignedTimepointIndices.pop_back();
        mAssignedFluentIndices.pop_back();

        // Skip timepoint-row that have already been handled
        if( mAssignedTimepoints[timepoint].second)
        {
            LOG_DEBUG_S << "Skipping already assigned waypoint: " << timepoint << " " << mAssignedTimepoints[timepoint].first;
            continue;
        }

        int offset = timepoint*mNumberOfFluents;
        int idx = offset + fluent;

        // This is an actual assigned waypoint
        // so update
        if(x[idx].lubSize() == 1)
        {
            int prev = idx - mNumberOfFluents;
            if(prev > 0)
            {
                // Constrain all columns of source nodes that lead to the current
                // node i, to the domain of the current node
                ModEvent ev = constrainSametimeView(home, prev, idx, idx);
                if(ev == Gecode::ME_GEN_FAILED)
                {
                    return ES_FAILED;
                }
            }
            // There can be only one --
            // which should not be needed since we have established the
            // linear constraint on cardinality already
            ModEvent ev = disableSametimeView(home, idx);
            if(ev == Gecode::ME_GEN_FAILED)
            {
                return ES_FAILED;
            }

            int targetIdx = x[idx].lubMax();
            // target may be empty
            ev = disableSametimeView(home, targetIdx);
            if(ev == Gecode::ME_GEN_FAILED)
            {
                return ES_FAILED;
            }

            mAssignedTimepoints[timepoint] = std::pair<int, bool>( idx, true);
        } else if (x[idx].lubSize() == 0)
        {
            if(timepoint > 0 && (size_t) timepoint + 1 < mNumberOfTimepoints)
            {
                excludeTarget(home, (timepoint-1)*mNumberOfFluents, idx);
            }
        }
    }

    size_t start, end;
    if(x.assigned())
    {
        if( !isValidWaypointSequence(mAssignedTimepoints, start, end, true) )
        {
            return ES_FAILED;
        }

    //    LOG_WARN_S << "Waypoints: " << waypointsToString()
    //        << std::endl
    //        << x;
        return home.ES_SUBSUMED(*this);
    } else {
        // the propagator will be scheduled if one of its views have been modified
        return ES_FIX;
    }
}

std::string IsPath::waypointsToString() const
{
    std::stringstream ss;
    ss << "[";
    for(auto pair : mAssignedTimepoints)
    {
        ss << "[" << pair.first << "," << pair.second << "]";
    }
    ss << "]";
    ss << utils::Formatter::toString(x, mNumberOfFluents);
    return ss.str();
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

