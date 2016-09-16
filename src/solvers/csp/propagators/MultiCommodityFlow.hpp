#ifndef TEMPL_SOLVERS_CSP_PROPAGATORS_MULTI_COMMODITY_FLOW_HPP
#define TEMPL_SOLVERS_CSP_PROPAGATORS_MULTI_COMMODITY_FLOW_HPP

#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>

#include <organization_model/OrganizationModel.hpp>
#include <organization_model/OrganizationModelAsk.hpp>
#include <templ/Role.hpp>

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

// MultiCommodityFlow computation
class MultiCommodityFlow : public Gecode::NaryPropagator<Gecode::Int::IntView, Gecode::Int::PC_INT_BND>
{
public:
    typedef Gecode::ViewArray<Gecode::Int::IntView> IntViewViewArray;
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
            IntViewViewArray& timelines,
            uint32_t numberOfTimepoints, uint32_t numberOfFluent,
            const organization_model::OrganizationModelAsk& ask);

    MultiCommodityFlow(Gecode::Space& home,
            bool share,
            MultiCommodityFlow& flow);

    static Gecode::ExecStatus post(Gecode::Space& home, const Role::List& roles,
            IntViewViewArray& timelines,
            uint32_t numberOfTimepoints, uint32_t numberOfFluent,
            const organization_model::OrganizationModelAsk& ask);

    virtual size_t dispose(Gecode::Space& home);

    virtual Gecode::Propagator* copy(Gecode::Space& home, bool share);

    virtual Gecode::PropCost cost(const Gecode::Space&, const Gecode::ModEventDelta&) const;

    virtual Gecode::ExecStatus propagate(Gecode::Space& home, const Gecode::ModEventDelta&);
protected:
    Role::List mRoles;
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;
    uint32_t mLocationTimeSize;
    organization_model::OrganizationModelAsk mAsk;

    std::vector<uint32_t> mCapacities;

    /**
     * Commodity flows
     * Each immobile uint is considered a commodity
     */
    std::vector<Gecode::IntVarArray> mCapacityFlowGraph;

    Gecode::IntVarArray mCapacityGraph;
    Gecode::IntVarArray mRemainingCapacityGraph;
    Gecode::IntVarArray mRoleCapacities;
};

void multiCommodityFlow(Gecode::Space& home,
        const Role::List& roles,
        const std::vector<Gecode::IntVarArray>& timelines,
        uint32_t numberOfTimepoints, uint32_t numberOfFluents,
        const organization_model::OrganizationModelAsk& ask);

} // end propagators
} // end csp
} // end solvers
} // end templ
#endif // TEMPL_SOLVERS_CSP_PROPAGATORS_MULTI_COMMODITY_FLOW_HPP
