#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>
#include <base-logging/Logging.hpp>
#include <organization_model/facets/Robot.hpp>

#include "MultiCommodityFlow.hpp"

using namespace Gecode;

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

void multiCommodityFlow(Gecode::Space& home,
        const Role::List& roles,
        const std::vector<Gecode::IntVarArray>& timelines,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        const organization_model::OrganizationModelAsk& ask)
{
    // If there is no path -- fail directly
    if(timelines.empty())
    {
        LOG_WARN_S << "TIMELINES EMPTY -- FAIL";
        home.fail();
    } else {
        // Construct as single ArrayView in order to perform the propagation
        // It concatenates all the timelines that exist for the given (active)
        // roles
        size_t viewIdx = 0;
        ViewArray<Int::IntView> xv(home, timelines.front().size()*roles.size());
        std::vector<Gecode::IntVarArray>::const_iterator tit = timelines.begin();
        for(; tit != timelines.end(); ++tit)
        {
            const Gecode::IntVarArray& args = *tit;
            Gecode::IntVarArray::const_iterator ait = args.begin();
            for(; ait != args.end(); ++ait)
            {
                //LOG_WARN_S << "ACCESS at: " << viewIdx << "size is " << xv.size();
                assert(viewIdx < xv.size());
                xv[viewIdx++] = Int::IntView(*ait);
            }
        }

        LOG_DEBUG_S << "MultiCommodityFlow: propagate #timepoints " << numberOfTimepoints << ", #fluents " << numberOfFluents;
        if(MultiCommodityFlow::post(home, roles, xv, numberOfTimepoints, numberOfFluents, ask) != ES_OK)
        {
            home.fail();
        }
    }

    if(home.failed())
    {
        return;
    }
}

MultiCommodityFlow::MultiCommodityFlow(Gecode::Space& home,
        const Role::List& roles,
        IntViewViewArray& xv,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        const organization_model::OrganizationModelAsk& ask)
    : NaryPropagator<Int::IntView, Int::PC_INT_BND>(home, xv)
    , mRoles(roles)
    , mNumberOfTimepoints(numberOfTimepoints)
    , mNumberOfFluents(numberOfFluents)
    , mLocationTimeSize(numberOfTimepoints*numberOfFluents)
    , mAsk(ask)
    , mCapacityGraph(home, (int) mLocationTimeSize, 0,Gecode::Int::Limits::max)
    , mRoleCapacities(home, mLocationTimeSize*roles.size(), Gecode::Int::Limits::min, Gecode::Int::Limits::max)
{
//    uint32_t numberOfSpaceTimePoints = mNumberOfTimepoints*mNumberOfFluents;
//    Gecode::Matrix<Gecode::IntVarArray> capacityGraph(mCapacityGraph,
//            numberOfSpaceTimePoints, // width --> col
//            numberOfSpaceTimePoints // height --> row
//            );
//
//    // Construct the basic graph that allows for transitions
//    // between space-time if the space (location) does not change at
//    // all
//    for(uint32_t row = 0; row < numberOfTimepoints; ++row)
//    {
//        for(uint32_t col = 0; col < numberOfSpaceTimePoints; ++col)
//        {
//            Gecode::IntVar transportCapacity = capacityGraph(col, row);
//            uint32_t capacity = 0;
//            // Allow a transition only from this to the next timepoint (no skipping)
//            // row is source, col is destination
//            if(col == row + 1)
//            {
//                // Make sure we are not at the start of a location sequence,
//                // i.e. anything with the initial timepoint
//                if( col%mNumberOfTimepoints != 0)
//                {
//                    capacity = Gecode::Int::Limits::max;
//                }
//            }
//            LOG_WARN_S << "BaseTimeline: row " << row << "col " << col << capacity;
//    //        rel(home, transportCapacity, Gecode::IRT_EQ, capacity);
//        }
//    }
//
//
//    // Cache supply/demand for each role to
//    // avoid recomputation
//    // positive value means a provided capacity
//    // a negative value means required (transport) capacity
//    std::vector<int32_t> mRoleSupplyDemand;
//    for(uint32_t roleIdx = 0; roleIdx < mRoles.size(); ++roleIdx)
//    {
//        const Role& role = mRoles[roleIdx];
//        organization_model::facets::Robot robot(role.getModel(), mAsk);
//        int32_t supplyDemand = robot.getPayloadTransportSupplyDemand();
//        mRoleSupplyDemand.push_back(supplyDemand);
//
//        LOG_WARN_S << "SupplyDemand: " << role.toString()   << " " << supplyDemand;
//    }
//
//    Gecode::IntConLevel intConLevel = Gecode::ICL_DEF;

    //for(uint32_t timelineEdgeIdx = 0;  timelineEdgeIdx < mLocationTimeSize; ++timelineEdgeIdx)
    //{
    //    Gecode::IntVar& baseCapacity = mCapacityGraph[timelineEdgeIdx];
    //    Gecode::LinIntExpr sumOfSupplyDemand = 0;
    //    sumOfSupplyDemand = sumOfSupplyDemand + baseCapacity;

    //    for(uint32_t roleIdx = 0; roleIdx < mRoles.size(); ++roleIdx)
    //    {
    //        int idx = mLocationTimeSize*roleIdx + timelineEdgeIdx;
    //        Gecode::Int::IntView& elementView = xv[idx];

    //        sumOfSupplyDemand = sumOfSupplyDemand + elementView;
    //    }

    //    Gecode::LinIntRel noNegativeCapacity(sumOfSupplyDemand, Gecode::IRT_GQ, 0);
    //    noNegativeCapacity.post(home, true, intConLevel);
    //}
}

MultiCommodityFlow::MultiCommodityFlow(Gecode::Space& home, bool share, MultiCommodityFlow& flow)
    : NaryPropagator<Int::IntView, Int::PC_INT_BND>(home, share, flow)
    , mAsk(flow.mAsk)
{
    x.update(home, share, flow.x);
}

Gecode::ExecStatus MultiCommodityFlow::post(Gecode::Space& home,
        const Role::List& roles,
        IntViewViewArray& xv,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        const organization_model::OrganizationModelAsk& ask
        )
{
    // documentation. 4.3.1 Post functions are clever
    // A constraint post function carefully analyzes its arguments. Based on
    // this analysis, the constraint post function chooses the best possible propagator for the constraint
    (void) new (home) MultiCommodityFlow(home, roles, xv, numberOfTimepoints, numberOfFluents, ask);
    return ES_OK;
}

size_t MultiCommodityFlow::dispose(Gecode::Space& home)
{
    (void) Propagator::dispose(home);
    return sizeof(*this);
}

Gecode::Propagator* MultiCommodityFlow::copy(Gecode::Space& home, bool share)
{
    return new (home) MultiCommodityFlow(home, share, *this);
}

Gecode::PropCost MultiCommodityFlow::cost(const Gecode::Space&, const Gecode::ModEventDelta&) const
{
    return Gecode::PropCost::quadratic(PropCost::HI, x.size());
}

Gecode::ExecStatus MultiCommodityFlow::propagate(Gecode::Space& home, const Gecode::ModEventDelta&)
{
    if(x.assigned())
    {
        LOG_WARN_S << "SUBSUMED";
        return home.ES_SUBSUMED(*this);
    } else {
        LOG_WARN_S << "NO FIX";
        // the propagator will be scheduled if one of its views have been modified
        return ES_NOFIX;
    }
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

