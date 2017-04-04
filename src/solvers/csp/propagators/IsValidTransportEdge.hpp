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
class IsValidTransportEdge : public Gecode::NaryPropagator<Gecode::Set::SetView, Gecode::Set::PC_SET_NONE>
{
public:
    typedef Gecode::ViewArray<Gecode::Set::SetView> SetVarArrayView;

    class DemandSupply : public Gecode::Advisor
    {
    public:
        // This represent the outgoing edges that have a demand and associated demands
        std::set<size_t> edgeIdxWithDemand;
        // This array represent the current supply demand on all edges
        std::vector<int> edgeValue;
        SetVarArrayView x;

        DemandSupply(Gecode::Space& home, Gecode::Propagator& p, Gecode::Council<DemandSupply>& c, SetVarArrayView x);
        DemandSupply(Gecode::Space& home, bool shared, DemandSupply& a);

        void dispose(Gecode::Space& home, Gecode::Council<DemandSupply>& c);

        std::string toString() const;

    };

    Gecode::Council<DemandSupply> c;

protected:
    size_t mTimepoint;
    size_t mFluent;

    std::vector<int32_t> mSupplyDemand;
    std::vector<size_t> mDemandEdges;
    uint32_t mLocalTargetFluent;
    uint32_t mSpaceTimeOffset;

public:
    /**
     * Checks if the given SetVarArray forms a valid muliEdge
     */
    IsValidTransportEdge(Gecode::Space& home, SetVarArrayView& multiEdge, const std::vector<int32_t>& supplyDemand, uint32_t timepoint, uint32_t fluent, uint32_t numberOfFluents);

    IsValidTransportEdge(Gecode::Space& home, bool share, IsValidTransportEdge& p);

    /**
     * IsPath propagators post function, i.e. when it is initially created
     */
    static Gecode::ExecStatus post(Gecode::Space& home, const Gecode::SetVarArgs& x0,
            const std::vector<int32_t>& supplyDemand,
            uint32_t timepoint,
            uint32_t fluent,
            uint32_t numberOfFluents);
    /**
     * Cancels that subscription of the view
     * \return the size of the just disposed propagator
     */
    virtual size_t dispose(Gecode::Space& home);
    virtual Gecode::Propagator* copy(Gecode::Space& home, bool share);
    virtual Gecode::PropCost cost(const Gecode::Space&, const Gecode::ModEventDelta&) const;
    virtual Gecode::ExecStatus advise(Gecode::Space& home, Gecode::Advisor& a, const Gecode::Delta& d);
    virtual void reschedule(Gecode::Space& home);
    virtual Gecode::ExecStatus propagate(Gecode::Space& home, const Gecode::ModEventDelta&);

    std::string toString() const;

};

void isValidTransportEdge(Gecode::Space& home, const Gecode::SetVarArgs&, const std::vector<int32_t>& supplyDemand, uint32_t timepoint, uint32_t fluent, uint32_t numberOfFluents);

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_PROPAGATORS_IS_VALID_TRANSPORT_EDGE_HPP
