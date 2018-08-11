#include "InEdgesRestriction.hpp"

#include <algorithm>
#include <gecode/int/rel.hh>
#include <gecode/set/rel.hh>
#include <gecode/minimodel.hh>
#include <base-logging/Logging.hpp>
#include <sstream>

using namespace Gecode;

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

void restrictInEdges(Gecode::Space& home, const std::vector<Gecode::SetVarArray>& x,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        size_t locationIdx,
        size_t minCount, size_t maxCount,
        const std::string& tag)
{

    LOG_INFO_S << "Restrict inedges for " << std::endl
        << "    # of timelines: " << x.size() << std::endl
        << "    min:" << minCount << std::endl
        << "    max:" << maxCount << std::endl
        << "    tag: " << tag << std::endl;

    // If there is no path -- fail directly
    if(x.size() == 0)
    {
        home.fail();
    } else {
        if(InEdgesRestriction::post(home, x, numberOfTimepoints, numberOfFluents,
                    locationIdx, minCount, maxCount,
                    tag) != ES_OK)
        {
            home.fail();
        }
    }

    if(home.failed())
    {
        return;
    }
}

InEdgesRestriction::InEdgesRestriction(Gecode::Space& home, ViewArray<Set::SetView>& xv,
        size_t numberOfTimelines,
        size_t numberOfTimepoints, size_t numberOfFluents,
        size_t locationIdx,
        size_t minCount, size_t maxCount,
        const std::string& tag)

    : NaryPropagator<Set::SetView, Set::PC_SET_NONE>(home, xv)
    , c(home)
    , mTag(tag)
    , mNumberOfTimelines(numberOfTimelines)
    , mNumberOfTimepoints(numberOfTimepoints)
    , mNumberOfFluents(numberOfFluents)
    , mLocationIdx(locationIdx)
    , mMinCount(minCount)
    , mMaxCount(maxCount)
    , mAssignedTimepoints()
{
    // If the home space is deleted this makes sure that the propagator will
    // also be deleted
    // see http://www.gecode.org/doc-latest/reference/group__TaskActor.html
    home.notice(*this, Gecode::AP_DISPOSE);

    assert(mNumberOfTimelines != 0);
    assert(mNumberOfTimepoints != 0);
    assert(mNumberOfFluents != 0);

    // Idx preparation
    // Group all space-time-points that refer to the same timepoint
    // in one group for observation
    for(size_t t = mNumberOfTimepoints; t--; )
    {
        // Collect all setvar that are associated with one timepoint
        SetVarArrayView arrayView(home, mNumberOfFluents*mNumberOfTimelines);

        size_t insideTimelineOffset = t*mNumberOfFluents;
        for(size_t l = mNumberOfFluents; l--; )
        {
            for(size_t r = mNumberOfTimelines; r--; )
            {
                size_t timelineSelectionOffset = r*mNumberOfFluents*mNumberOfTimepoints;
                size_t idx = timelineSelectionOffset + insideTimelineOffset + l;

                arrayView[r*mNumberOfFluents + l] = x[idx];
            }
        }
        // Create array view for timepoints
        (void) new (home) Idx(home, *this, c, t, true, arrayView);
    }

    reschedule(home);
}

InEdgesRestriction::InEdgesRestriction(Gecode::Space& home, InEdgesRestriction& p)
    : NaryPropagator<Set::SetView, Set::PC_SET_NONE>(home, p)
    , mTag(p.mTag)
    , mNumberOfTimelines(p.mNumberOfTimelines)
    , mNumberOfTimepoints(p.mNumberOfTimepoints)
    , mNumberOfFluents(p.mNumberOfFluents)
    , mLocationIdx(p.mLocationIdx)
    , mMinCount(p.mMinCount)
    , mMaxCount(p.mMaxCount)
    , mAssignedTimepoints(p.mAssignedTimepoints)
{
    x.update(home, p.x);
    c.update(home, p.c);
}

Gecode::ExecStatus InEdgesRestriction::post(Gecode::Space& home,
        const std::vector<Gecode::SetVarArray>& xArray,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        size_t locationIdx,
        size_t minCount, size_t maxCount,
        const std::string& tag)
{
    ViewArray<Set::SetView> viewArray(home, xArray.size()*numberOfTimepoints*numberOfFluents);

    size_t idx = 0;
    for(const Gecode::SetVarArray& a : xArray)
    {
        for(size_t i = 0; i < numberOfTimepoints*numberOfFluents; ++i)
        {
            viewArray[idx] = Gecode::Set::SetView(a[i]);
            ++idx;
        }
    }

    // documentation. 4.3.1 Post functions are clever
    // A constraint post function carefully analyzes its arguments. Based on
    // this analysis, the constraint post function chooses the best possible propagator for the constraint
    (void) new (home) InEdgesRestriction(home,
            viewArray,
            xArray.size(),
            numberOfTimepoints, numberOfFluents,
            locationIdx,
            minCount, maxCount,
            tag);
    return ES_OK;
}

size_t InEdgesRestriction::dispose(Gecode::Space& home)
{
    home.ignore(*this, AP_DISPOSE);
    c.dispose(home);
    (void) NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_NONE>::dispose(home);
    return sizeof(*this);
}

Gecode::Propagator* InEdgesRestriction::copy(Gecode::Space& home)
{
    return new (home) InEdgesRestriction(home, *this);
}

Gecode::PropCost InEdgesRestriction::cost(const Gecode::Space&, const Gecode::ModEventDelta&) const
{
    return Gecode::PropCost::quadratic(PropCost::LO, x.size());
}

void InEdgesRestriction::reschedule(Gecode::Space& home)
{
    Gecode::Set::SetView::schedule(home, *this, Gecode::ME_GEN_ASSIGNED);
}

Gecode::ExecStatus InEdgesRestriction::advise(Gecode::Space& home, Gecode::Advisor& a, const Gecode::Delta& d)
{
    ModEvent me = Gecode::Set::SetView::modevent(d);
    if(me == Gecode::ME_GEN_ASSIGNED)
    {
        Idx& advisor = static_cast<Idx&>(a);
        int idx = advisor.idx();
        // Only trigger when the advisors observed
        // variables are all assigned
        if(advisor.x.assigned())
        {
            if(advisor.isTimepointIdx())
            {
                mAssignedTimepointIndices.insert(idx);
            }
            return home.ES_NOFIX_DISPOSE(c, advisor);
        }
    }
    // Do not schedule unless a view has been assigned
    return ES_FIX;
}

Gecode::ExecStatus InEdgesRestriction::propagate(Gecode::Space& home, const Gecode::ModEventDelta&)
{
    while(!mAssignedTimepointIndices.empty())
    {
        int timepoint = *mAssignedTimepointIndices.begin();
        mAssignedTimepointIndices.erase(mAssignedTimepointIndices.begin());
        size_t inEdgeCount = 0;
        for(size_t r = 0; r < mNumberOfTimelines; ++r)
        {
            int offset = r*mNumberOfFluents*mNumberOfTimepoints +
                timepoint*mNumberOfFluents;
            for(size_t l = 0; l < mNumberOfFluents; ++l)
            {
                if(l == mLocationIdx)
                {
                    // self references are allowed
                } else {
                    const Gecode::Set::SetView& s = x[offset + l];
                    if(s.lubSize() == 1)
                    {
                        int targetIdx = s.lubMax();
                       if( targetIdx % mNumberOfFluents == mLocationIdx)
                        {
                            inEdgeCount += 1;
                        }
                    }
                }
            }
            mAssignedTimepoints[timepoint] = inEdgeCount;
        }
    }

    // Allow to abort early when the maximum is exceeded
    size_t accumulatedInEdgesCount = 0;
    for(const std::pair<int,size_t>& p : mAssignedTimepoints)
    {
        accumulatedInEdgesCount += p.second;
    }

    if(accumulatedInEdgesCount > mMaxCount)
    {
        return ES_FAILED;
    }

    if(x.assigned())
    {
        if(accumulatedInEdgesCount < mMinCount)
        {
            return ES_FAILED;
        }
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


