#include "IsValidTransportEdge.hpp"
#include <base/Logging.hpp>

using namespace Gecode;

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

IsValidTransportEdge::DemandSupply::DemandSupply(Gecode::Space& home,
        Gecode::Propagator& p,
        Gecode::Council<DemandSupply>& c,
        SetVarArrayView x,
        size_t numberOfFluents)
    : Gecode::Advisor(home, p, c)
    , edgeValue(numberOfFluents, 0)
    , x(x)
{
    x.subscribe(home, *this);
}

IsValidTransportEdge::DemandSupply::DemandSupply(Gecode::Space& home, bool shared, DemandSupply& other)
    : Gecode::Advisor(home, shared, other)
    , edgeIdxWithDemand(other.edgeIdxWithDemand)
    , edgeValue(other.edgeValue)
{
    x.update(home, shared, other.x);
}

void IsValidTransportEdge::DemandSupply::dispose(Gecode::Space& home, Gecode::Council<DemandSupply>& c)
{
    x.cancel(home, *this);
    Advisor::dispose(home, c);
}

std::string IsValidTransportEdge::DemandSupply::toString() const
{
    std::stringstream ss;
    ss << "EdgeIdxWithDemand: [";
    std::set<size_t>::const_iterator cit = edgeIdxWithDemand.begin();
    for(; cit != edgeIdxWithDemand.end();)
    {
        ss << *cit;
        ++cit;

        if(cit != edgeIdxWithDemand.end())
        {
            ss << ", ";
        }
    }
    ss << "]" << std::endl;
    ss << "EdgeValue: [";
    for(size_t i = 0; i < edgeValue.size(); ++i)
    {
        ss << edgeValue[i];
        if(i < edgeValue.size() - 1 )
        {
           ss << ", ";
        }
    }
    ss << "]" << std::endl;
    ss << "raw: " << x;
    return ss.str();
}

IsValidTransportEdge::IsValidTransportEdge(Gecode::Space& home, SetVarArrayView& multiEdge, const std::vector<int32_t>& supplyDemand, uint32_t timepoint, uint32_t fluent, uint32_t numberOfFluents)
    : NaryPropagator<Gecode::Set::SetView, Set::PC_SET_NONE>(home, multiEdge)
    , c(home)
    , mTimepoint(timepoint)
    , mFluent(fluent)
    , mSupplyDemand(supplyDemand)
    , mLocalTargetFluent((timepoint+1)*numberOfFluents + fluent)
    , mSpaceTimeOffset((timepoint+1)*numberOfFluents)
{
    // If the home space is deleted this makes sure that the propagator will
    // also be deleted
    // see http://www.gecode.org/doc-latest/reference/group__TaskActor.html
    home.notice(*this, Gecode::AP_DISPOSE);

    assert(mLocalTargetFluent != 0);
    assert(x.size() == static_cast<int>(mSupplyDemand.size()) );

    (void) new (home) DemandSupply(home, *this, c, x, numberOfFluents);
}

IsValidTransportEdge::IsValidTransportEdge(Gecode::Space& home, bool share, IsValidTransportEdge& p)
    : NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_NONE>(home, share, p)
    , mTimepoint(p.mTimepoint)
    , mFluent(p.mFluent)
    , mSupplyDemand(p.mSupplyDemand)
    , mLocalTargetFluent(p.mLocalTargetFluent)
    , mSpaceTimeOffset(p.mSpaceTimeOffset)
{
    x.update(home, share, p.x);
    c.update(home, share, p.c);
}

 Gecode::ExecStatus IsValidTransportEdge::post(Gecode::Space& home, const Gecode::SetVarArgs& multiEdge,
            const std::vector<int32_t>& supplyDemand,
            uint32_t timepoint,
            uint32_t fluent,
            uint32_t numberOfFluents)
{
    bool hasDemand = false;
    for(int i = 0; i < multiEdge.size(); ++i)
    {
        if(supplyDemand[i] < 0)
        {
            if(multiEdge[i].lubSize() != 0)
            {
                hasDemand = true;
                break;
            }
        }
    }

    // Unless there is a particular demand we can safely ignore this
    // constraint
    if(hasDemand)
    {
        ViewArray<Set::SetView> viewArray(home, multiEdge);
        (void) new (home) IsValidTransportEdge(home, viewArray, supplyDemand, timepoint, fluent, numberOfFluents);
    }
    return ES_OK;
}

size_t IsValidTransportEdge::dispose(Gecode::Space& home)
{
    home.ignore(*this, AP_DISPOSE);
    c.dispose(home);
    (void) NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_NONE>::dispose(home);
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
    if(x.assigned())
    {
        size_t size = this->dispose(home);
        return home.ES_SUBSUMED_DISPOSED(*this, size);
    }
    return ES_FAILED;
}

void isValidTransportEdge(Gecode::Space& home, const Gecode::SetVarArgs& x, const std::vector<int32_t>& supplyDemand, uint32_t timepoint, uint32_t fluent, uint32_t numberOfFluents)
{
    // No edges
    if(x.size() == 0)
    {
        home.fail();
    } else {
        if(IsValidTransportEdge::post(home, x, supplyDemand, timepoint, fluent, numberOfFluents) != ES_OK)
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

Gecode::ExecStatus IsValidTransportEdge::advise(Gecode::Space& home, Gecode::Advisor& a, const Gecode::Delta& d)
{
    ModEvent me = Gecode::Set::SetView::modevent(d);
    if(me == Gecode::ME_GEN_ASSIGNED)
    {
        DemandSupply& advisor = static_cast<DemandSupply&>(a);

        if(x.assigned())
        {
            for(size_t idx = 0; idx < (size_t) x.size(); ++idx)
            {
                Gecode::Set::SetView& view = advisor.x[idx];
                if(view.lubSize() == 1)
                {
                    // Get the local target edge idx
                    int targetIdx = view.lubMax();
                    // assumption of forward pointing in time
                    assert(targetIdx >= mSpaceTimeOffset);
                    size_t localTargetEdgeIdx = targetIdx - mSpaceTimeOffset;

                    // Mark edges that have an actual demand
                    if(mSupplyDemand[idx] < 0)
                    {
                        if( static_cast<size_t>( targetIdx ) != mLocalTargetFluent + idx)
                        {
                            advisor.edgeIdxWithDemand.insert(localTargetEdgeIdx);
                        }
                    }
                    advisor.edgeValue[localTargetEdgeIdx] += mSupplyDemand[idx];
                }
            }

            std::set<size_t>::const_iterator cit = advisor.edgeIdxWithDemand.begin();
            for(; cit != advisor.edgeIdxWithDemand.end(); ++cit)
            {
                assert(advisor.edgeValue.size() > *cit);
                int demand = advisor.edgeValue.at( *cit );
                if(demand < 0)
                {
                    return ES_FAILED;
                }
            }
            return home.ES_NOFIX_DISPOSE(c, advisor);
        }
    }
    return ES_FIX;
}

std::string IsValidTransportEdge::toString() const
{
    std::stringstream ss;

    ss << "TransportEdge:" << std::endl;
    ss << "    Timepoint: " << mTimepoint << std::endl;
    ss << "    Fluent:    " << mFluent << std::endl;

    return ss.str();
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

