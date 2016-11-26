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
        const std::vector<Gecode::SetVarArray>& timelines,
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
        ViewArray<Set::SetView> xv(home, timelines.front().size()*roles.size());
        std::vector<Gecode::SetVarArray>::const_iterator tit = timelines.begin();
        for(; tit != timelines.end(); ++tit)
        {
            const Gecode::SetVarArray& args = *tit;
            Gecode::SetVarArray::const_iterator ait = args.begin();
            for(; ait != args.end(); ++ait)
            {
                assert(viewIdx < xv.size());
                xv[viewIdx++] = Set::SetView(*ait);
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
        SetViewViewArray& xv,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        const organization_model::OrganizationModelAsk& ask)
    : NaryPropagator<Set::SetView, Set::PC_SET_ANY>(home, xv)
    , mRoles(roles)
    , mNumberOfTimepoints(numberOfTimepoints)
    , mNumberOfFluents(numberOfFluents)
    , mLocationTimeSize(numberOfTimepoints*numberOfFluents)
    , mTimelineSize(mLocationTimeSize*mLocationTimeSize)
    , mAsk(ask)
    , mCapacityGraph()
{
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
            if(supplyDemand == 0)
            {
                throw std::invalid_argument("templ::propagators::MultiCommodityFlow: " +  role.getModel().toString() + "has"
                        " a transportSupplyDemand of 0 -- must be either positive of negative integer");
            }
            mRoleSupplyDemand.push_back(supplyDemand);

            LOG_WARN_S << "SupplyDemand: " << role.toString()   << " " << supplyDemand;

            for(uint32_t i = 0; i < mLocationTimeSize; ++i)
            {
                // the current roles timeline is in the
                // concatenated list of timelines, thus we use
                // the start offset: roleIdx*mLocationTimeSize
                // to then interate over the timeline values
                size_t idx = roleIdx*mLocationTimeSize + i;

                // the target index (in the local timeline)
                // -- the set is actually an adjacency list, so the index
                // identifies the source space-time and the set value the
                // target's space-time (if there is a target)
                SetVar var(xv[idx]);
                if(!var.assigned())
                {
                    throw std::invalid_argument("Cannot propagate since value is not assigned");
                }

                LOG_INFO_S << "Role " << roleIdx << " pos: " << i << " val glbSize: " << var.glbSize() << "  lubSize: " << var.lubSize() << " cardMax: " << var.cardMax() << " cardMin: " << var.cardMin();
                // Check that the value is assigned, by checking that
                // the set has cardinality 1
                if(var.cardMax() == 1 && var.cardMin() == 1)
                {
                    Gecode::SetVarGlbValues currentVar(var);
                    std::pair<uint32_t, uint32_t> key(i, currentVar.val());

                    // here we assign the supply demand of the role to the
                    // current timeline value -- i will identify the same
                    // space-time in all role timelines
                    std::vector<int32_t>& supplyDemandVector = mCapacityGraph[key];
                    supplyDemandVector.push_back( supplyDemand );
                }
            }
        }
    }

    CapacityGraph::const_iterator cit = mCapacityGraph.begin();
    for(; cit != mCapacityGraph.end(); ++cit)
    {
        const CapacityGraphKey& key = cit->first;
        const std::vector<int32_t>& supplyDemand = cit->second;
        if(isLocalTransition(key))
        {
            // this is a local transition with no capacity restriction
            LOG_WARN_S << "This is a local transition: from " << key.first << " to " << key.second;
        } else {
            int32_t sum = 0;
            std::vector<int32_t>::const_iterator vit = supplyDemand.begin();
            for(; vit != supplyDemand.end(); ++vit)
            {
                sum += *vit;
            }
            if(sum < 0)
            {
                LOG_WARN_S << "SUM is " << sum << "  failing space at transition: from " <<  key.first << " to " << key.second;
                LOG_WARN_S << xv;
                home.fail();
            }
        }

    }
}

MultiCommodityFlow::MultiCommodityFlow(Gecode::Space& home, bool share, MultiCommodityFlow& flow)
    : NaryPropagator<Set::SetView, Set::PC_SET_ANY>(home, share, flow)
    , mAsk(flow.mAsk)
    , mCapacityGraph(flow.mCapacityGraph)
    , mRoleSupplyDemand(flow.mRoleSupplyDemand)
{
    x.update(home, share, flow.x);
}

Gecode::ExecStatus MultiCommodityFlow::post(Gecode::Space& home,
        const Role::List& roles,
        SetViewViewArray& xv,
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

bool MultiCommodityFlow::isLocalTransition(const CapacityGraphKey& key) const
{
    if (key.second - key.first == mNumberOfFluents)
    {
        return true;
    }
    return false;
}

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

