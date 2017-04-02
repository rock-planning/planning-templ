#include "IsValidTransportEdge.hpp"
#include <base/Logging.hpp>

using namespace Gecode;

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {


IsValidTransportEdge::IsValidTransportEdge(Gecode::Space& home, SetVarArrayView& multiEdge, const std::vector<int32_t>& supplyDemand, uint32_t localTargetFluent)
    : NaryPropagator<Gecode::Set::SetView, Set::PC_SET_VAL>(home, multiEdge)
    , mSupplyDemand(supplyDemand)
    , mLocalTargetFluent(localTargetFluent)
{
    // If the home space is deleted this makes sure that the propagator will
    // also be deleted
    // see http://www.gecode.org/doc-latest/reference/group__TaskActor.html
    home.notice(*this, Gecode::AP_DISPOSE);

    assert(mLocalTargetFluent != 0);
    assert(x.size() == static_cast<int>(mSupplyDemand.size()) );

    LOG_WARN_S << "Create transport edge: " << multiEdge;
}

IsValidTransportEdge::IsValidTransportEdge(Gecode::Space& home, bool share, IsValidTransportEdge& p)
    : NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_VAL>(home, share, p)
    , mSupplyDemand(p.mSupplyDemand)
    , mLocalTargetFluent(p.mLocalTargetFluent)
{
    x.update(home, share, p.x);
}

 Gecode::ExecStatus IsValidTransportEdge::post(Gecode::Space& home, const Gecode::SetVarArgs& multiEdge,
            const std::vector<int32_t>& supplyDemand,
            uint32_t localTargetFluent)
{
    ViewArray<Set::SetView> viewArray(home, multiEdge);
    (void) new (home) IsValidTransportEdge(home, viewArray, supplyDemand, localTargetFluent);
    return ES_OK;
}

size_t IsValidTransportEdge::dispose(Gecode::Space& home)
{
    home.ignore(*this, AP_DISPOSE);
    (void) NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_VAL>::dispose(home);
    return sizeof(*this);
}

Gecode::Propagator* IsValidTransportEdge::copy(Gecode::Space& home, bool share)
{
    return new (home) IsValidTransportEdge(home, share, *this);
}

Gecode::PropCost IsValidTransportEdge::cost(const Gecode::Space&, const Gecode::ModEventDelta&) const
{
    return Gecode::PropCost::linear(PropCost::LO, x.size());
}

void IsValidTransportEdge::reschedule(Gecode::Space& home)
{
    Gecode::Set::SetView::schedule(home, *this, Gecode::ME_GEN_ASSIGNED);
}

Gecode::ExecStatus IsValidTransportEdge::propagate(Gecode::Space& home, const Gecode::ModEventDelta&)
{
    // TODO: reduce check to path segments of immobile systems, i.e.
    // when this is critical
    if(x.assigned())
    {
        LOG_WARN_S << "Assignment: " << x;
        std::vector<int> demandEdge;
        for(size_t a = 0; a < x.size(); ++a)
        {
            if(x[a].lubSize() == 1 && mSupplyDemand[a] < 0)
            {
                if(x[a].lubMax() != (mLocalTargetFluent + a))
                {
                    demandEdge.push_back(x[a].lubMax());
                } else {
                    LOG_WARN_S << "Is local edge: " << (mLocalTargetFluent + a);
                    LOG_WARN_S << "Offset starts at: " << mLocalTargetFluent;
                }
            }
        }

        if(demandEdge.empty())
        {
            return home.ES_SUBSUMED(*this);
        }

        for(int d = 0; d < demandEdge.size(); ++d)
        {
            int sum = 0;
            for(size_t a = 0; a < x.size(); ++a)
            {
                if(x[a].lubSize() == 0)
                {
                    continue;
                }

                int targetIdx = x[a].lubMax();
                // Local edge -- thus skipping
                if(targetIdx != demandEdge[d])
                {
                    continue;
                }

                sum += mSupplyDemand[a];
                //for(size_t b = a + 1; b < x.size(); ++b)
                //{
                //    if(x[b].lubSize() == 0)
                //    {
                //        continue;
                //    }

                //    int otherTargetIdx = x[b].lubMax();
                //    if(targetIdx == otherTargetIdx)
                //    {
                //        sum += mSupplyDemand[b];
                //    }
                //}
            }

            if(sum <= 0)
            {
                LOG_WARN_S << "Demand to " << demandEdge[d] << " in " << x << "  -- demand cannot be satisfied, sum of edges is: " << sum;
                return ES_FAILED;
            } else {
                LOG_WARN_S << "Demand to " << demandEdge[d] << " in " << x << "  -- demand can be satisfied, sum of edges is: " << sum;
            }
        }
        return home.ES_SUBSUMED(*this);
    } else {
        return ES_FIX;
    }
}

void isValidTransportEdge(Gecode::Space& home, const Gecode::SetVarArgs& x, const std::vector<int32_t>& supplyDemand, uint32_t localTargetFluent)
{
    // No edges
    if(x.size() == 0)
    {
        home.fail();
    } else {
        if(IsValidTransportEdge::post(home, x, supplyDemand, localTargetFluent) != ES_OK)
        {
            // If propagator fails we have no solution
            home.fail();
        }
    }
    if(home.failed())
    {
        return;
    }
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

