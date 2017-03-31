#ifndef TEMPL_SOLVERS_CSP_PROPAGATORS_IS_VALID_TRANSPORT_EDGE_HPP
#define TEMPL_SOLVERS_CSP_PROPAGATORS_IS_VALID_TRANSPORT_EDGE_HPP

#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/minimodel.hh>
#include <set>
#include <vector>

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

/**
 * Check if a given set is a consistent representation of a path
 * role0: t0-l0 --> [ t1-l1, ... ]
 * role1: t0-l0 --> [ t1-l1, ... ]
 */
class IsValidTransportEdge : public Gecode::NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_VAL>
{
public:
    typedef Gecode::ViewArray<Gecode::Set::SetView> SetVarArrayView;

protected:
    std::vector<int32_t> mSupplyDemand;
    uint32_t mLocalTargetFluent;

public:
    /**
     * Checks if the given SetVarArray forms a valid muliEdge
     */
    IsValidTransportEdge(Gecode::Space& home, SetVarArrayView& multiEdge, const std::vector<int32_t>& supplyDemand, uint32_t localTargetFluent);

    IsValidTransportEdge(Gecode::Space& home, bool share, IsValidTransportEdge& p);

    /**
     * IsPath propagators post function, i.e. when it is initially created
     */
    static Gecode::ExecStatus post(Gecode::Space& home, const Gecode::SetVarArgs& x0,
            const std::vector<int32_t>& supplyDemand,
            uint32_t localTargetFluent);
    /**
     * Cancels that subscription of the view
     * \return the size of the just disposed propagator
     */
    virtual size_t dispose(Gecode::Space& home);
    virtual Gecode::Propagator* copy(Gecode::Space& home, bool share);
    virtual Gecode::PropCost cost(const Gecode::Space&, const Gecode::ModEventDelta&) const;
    virtual void reschedule(Gecode::Space& home);
    virtual Gecode::ExecStatus propagate(Gecode::Space& home, const Gecode::ModEventDelta&);

};

void isValidTransportEdge(Gecode::Space& home, const Gecode::SetVarArgs&, const std::vector<int32_t>& supplyDemand, uint32_t localTargetFluent);

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_PROPAGATORS_IS_VALID_TRANSPORT_EDGE_HPP
