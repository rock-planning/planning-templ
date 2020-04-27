#ifndef TEMPL_SOLVERS_CSP_PROPAGATORS_MULTI_COMMODITY_FLOW_HPP
#define TEMPL_SOLVERS_CSP_PROPAGATORS_MULTI_COMMODITY_FLOW_HPP

#include <gecode/set.hh>
#include <gecode/set/rel.hh>
#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>

#include <organization_model/OrganizationModel.hpp>
#include <organization_model/OrganizationModelAsk.hpp>
#include "../../../Role.hpp"

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

// MultiCommodityFlow computation
class MultiCommodityFlow : public Gecode::NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_ANY>
{
public:
    typedef Gecode::ViewArray<Gecode::Set::SetView> SetViewViewArray;
    typedef std::pair<uint32_t,uint32_t> CapacityGraphKey;
    typedef std::map< CapacityGraphKey , std::vector<int32_t> > CapacityGraph;

    // Input
    //     --> TupleSet: defining what types of Systems of combinations are
    // required
    //     --> ModelRequirement: defining what models are at minimum required at
    //     one position
    //
    // Output:
    //     --> propagation of boolean constraints of the form
    //     Gecode::LinIntExpr cardinalityRequirementA(numberOfModelsA, Gecode::IRT_LE, requiredNumberOfModelAInstances);
    //     Gecode::LinIntExpr cardinalityRequirementB(numberOfModelsB, Gecode::IRT_LE, requiredNumberOfModelBInstances);
    //
    //     BoolExpr combinedSystemA(cardinalityRequirementA, BoolExpr::AND,
    //     cardinalityRequirementB);
    //
    //     BoolExpr minOneCombinedSystem(combinedSystemA, BoolExpr::OR,
    //     cardinalityRequirementB);


    MultiCommodityFlow(Gecode::Space& home, const Role::List& roles,
            SetViewViewArray& timelines,
            uint32_t numberOfTimepoints, uint32_t numberOfFluent,
            const organization_model::OrganizationModelAsk& ask);

    MultiCommodityFlow(Gecode::Space& home,
            MultiCommodityFlow& flow);

    static Gecode::ExecStatus post(Gecode::Space& home, const Role::List& roles,
            SetViewViewArray& timelines,
            uint32_t numberOfTimepoints, uint32_t numberOfFluent,
            const organization_model::OrganizationModelAsk& ask);

    virtual size_t dispose(Gecode::Space& home);

    virtual Gecode::Propagator* copy(Gecode::Space& home);

    virtual Gecode::PropCost cost(const Gecode::Space&, const Gecode::ModEventDelta&) const;

    virtual Gecode::ExecStatus propagate(Gecode::Space& home, const Gecode::ModEventDelta&);

    bool isLocalTransition(const CapacityGraphKey& key) const;
protected:
    Role::List mRoles;
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;
    uint32_t mLocationTimeSize;
    uint32_t mTimelineSize;

    organization_model::OrganizationModelAsk mAsk;

    // The elements in the wrapped array a are accessed in row-major order
    // as it is for all array in Gecode
    // http://www.gecode.org/doc-latest/reference/classGecode_1_1Matrix.html#_details
    CapacityGraph mCapacityGraph;

    // Map the role index to the transport supply/demand
    // Supply Demand can be either positive or negative
    std::vector<int32_t> mRoleSupplyDemand;
};

void multiCommodityFlow(Gecode::Space& home,
        const Role::List& roles,
        const std::vector<Gecode::SetVarArray>& timelines,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        const organization_model::OrganizationModelAsk& ask);

} // end propagators
} // end csp
} // end solvers
} // end templ
#endif // TEMPL_SOLVERS_CSP_PROPAGATORS_MULTI_COMMODITY_FLOW_HPP
