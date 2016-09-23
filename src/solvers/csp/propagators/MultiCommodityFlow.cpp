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
    , mTimelineSize(mLocationTimeSize*mLocationTimeSize)
    , mAsk(ask)
    , mCapacityGraph()
{
    {
    std::stringstream ss;
    ss << std::endl << "Number of Fluents: " << mNumberOfFluents << std::endl;

    // Construct the basic graph that allows for transitions (has infinite capacity)
    // between space-time if the fluent (e.g. location) does not change
    for(uint32_t row = 0; row < mLocationTimeSize; ++row)
    {
        for(uint32_t col = 0; col < mLocationTimeSize; ++col)
        {
            int32_t capacity = 0;
            // Allow a transition only from this to the next timepoint (no skipping)
            // row is source, col is destination, thus
            // from row to col == from row to row +1
            if(col == row + mNumberOfFluents)
            {
                capacity = 1;
            }
            ss << capacity << " ";
            mCapacityGraph.push_back(capacity);
        }
        ss << std::endl;
    }
    LOG_WARN_S << ss.str();
    }

    // Cache supply/demand for each role to
    // avoid recomputation
    // positive value means a provided capacity
    // a negative value means required (transport) capacity
    if(mRoleSupplyDemand.empty())
    {
        for(uint32_t roleIdx = 0; roleIdx < mRoles.size(); ++roleIdx)
        {
            const Role& role = mRoles[roleIdx];
            organization_model::facets::Robot robot(role.getModel(), mAsk);
            int32_t supplyDemand = robot.getPayloadTransportSupplyDemand();
            mRoleSupplyDemand.push_back(supplyDemand);

            LOG_WARN_S << "SupplyDemand: " << role.toString()   << " " << supplyDemand;
        }
    }

    Gecode::IntConLevel intConLevel = Gecode::ICL_DEF;

    std::stringstream ss;
    ss << std::endl;
    uint32_t timelineSize = mLocationTimeSize*mLocationTimeSize;
    for(uint32_t timelineEdgeIdx = 0;  timelineEdgeIdx < timelineSize; ++timelineEdgeIdx)
    {
            std::stringstream elements;
            Gecode::LinIntExpr sumOfSupplyDemand = 0;
            for(uint32_t roleIdx = 0; roleIdx < mRoles.size(); ++roleIdx)
            {
                int idx = timelineSize*roleIdx + timelineEdgeIdx;
                Gecode::Int::IntView elementView = xv[idx];

                sumOfSupplyDemand = sumOfSupplyDemand + elementView*mRoleSupplyDemand[roleIdx];
                elements << elementView;

                if(roleIdx < mRoles.size()-1)
                {
                    elements << "/";
                }
            }

            if(!isLocalTransition(timelineEdgeIdx))
            {
                Gecode::LinIntRel balancedCapacity(sumOfSupplyDemand, Gecode::IRT_GQ, 0);
                balancedCapacity.post(home, true, intConLevel);
            }

            if(timelineEdgeIdx%mLocationTimeSize != 0)
            {
                size_t rest = 24-elements.str().size();
                std::string hspace(rest,' ');
                ss << hspace;
            }
            ss << elements.str();

            if((timelineEdgeIdx+1)%mLocationTimeSize == 0)
            {
                ss << std::endl;
            }

    }
    LOG_WARN_S << ss.str();
}

MultiCommodityFlow::MultiCommodityFlow(Gecode::Space& home, bool share, MultiCommodityFlow& flow)
    : NaryPropagator<Int::IntView, Int::PC_INT_BND>(home, share, flow)
    , mAsk(flow.mAsk)
    , mCapacityGraph(flow.mCapacityGraph)
    , mRoleSupplyDemand(flow.mRoleSupplyDemand)
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
        return home.ES_SUBSUMED(*this);
    } else if(home.failed())
    {
        return ES_FAILED;
    } else {
        // the propagator will be scheduled if one of its views have been modified
        return ES_NOFIX;
    }
}

bool MultiCommodityFlow::isLocalTransition(uint32_t timelineEdgeIdx) const
{
    if (mCapacityGraph[timelineEdgeIdx] != 0)
    {
        return true;
    }
    return false;
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

