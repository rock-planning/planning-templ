#ifndef TEMPL_SOLVERS_CSP_TEMPORAL_CSP_PROPAGATORS_HPP
#define TEMPL_SOLVERS_CSP_TEMPORAL_CSP_PROPAGATORS_HPP

#include <gecode/int.hh>
#include <base/Logging.hpp>

namespace Gecode {

class PASymmetry : public BinaryPropagator<Int::IntView, Int::PC_INT_DOM>
{
protected:
    Int::IntView x0;
    Int::IntView x1;

public:
    PASymmetry(Home home, Int::IntView x0, Int::IntView x1)
        : BinaryPropagator<Int::IntView, Int::PC_INT_DOM>(home, x0, x1)
        , x0(x0)
        , x1(x1)
    {
        LOG_WARN_S << "Default constructor PASymmetry" << x1 << " " << x1;
        x0.subscribe(home, *this, Int::PC_INT_DOM);
        x1.subscribe(home, *this, Int::PC_INT_DOM);
    }

    PASymmetry(Space& home, bool share, PASymmetry& p)
        : BinaryPropagator<Int::IntView, Int::PC_INT_DOM>(home, share, p)
    {
        x0.update(home, share, p.x0);
        x1.update(home, share, p.x1);

        LOG_WARN_S << "Instanciate from copy constructor" << p.x0 << " " << p.x1 << std::endl
            << x0 << " " << x1;

    }

    virtual PropCost cost(const Space&, const ModEventDelta& ) const
    {
        LOG_WARN_S << "Cost PASymmetry";
        return PropCost::binary(PropCost::LO);
    }

    static ExecStatus post(Space& home, Int::IntView x0, Int::IntView x1)
    {
        LOG_WARN_S << "Posting PASymmetry " << x0 << " " << x1;
        (void) new (home) PASymmetry(home, x0, x1);
        return ES_OK;
    }

    virtual ExecStatus propagate(Space& home, const ModEventDelta&)
    {
        LOG_WARN_S << "Propagate PASymmetry: " << x0 << " " << x1 << "(size: " << x0.size() << ")";
        assert(x0.size() > 0);

        // Create domain for symmetric relationship
        int range[3];
        int index = 0;
        for(IntVarValues i(x0); i(); ++i, ++index)
        {
            range[index] = - i.val(); 
        }
        IntSet x_set(range,index);
        IntVar x_domain(home, x_set);
        LOG_WARN_S << " --- new domain: " << x_domain;

        IntVarValues x_values(x_domain);
        GECODE_ME_CHECK( x1.inter_v(home,x_values , true) );

        for(IntVarValues i(x0); i(); ++i)
        {
            LOG_WARN_S << "VALUE x0: " << i.val();
        }

        bool assigned = x0.assigned() && x1.assigned();
        return assigned ? home.ES_SUBSUMED(*this) : ES_NOFIX;

    }

    virtual size_t dispose(Space& home) 
    {
        LOG_WARN_S << "Dispose PASymmetry: " << x0 << " " << x1;
        x0.cancel(home, *this, Int::PC_INT_DOM);
        x1.cancel(home, *this, Int::PC_INT_DOM);

        (void) BinaryPropagator<Int::IntView, Int::PC_INT_DOM>::dispose(home);
        return sizeof(*this);
    }

    virtual Propagator* copy(Space& home, bool share)
    {
        return new (home) PASymmetry(home, share, *this);
    }

}; // end PASymmetry

class PathConsistency : public TernaryPropagator<Int::IntView, Int::PC_INT_DOM>
{
protected:
    Int::IntView x_st;
    Int::IntView x_so;
    Int::IntView x_ot;

public:
    // Int::PC_INT_DOM is a propagation condition --> section 22.6
    PathConsistency(Home home, Int::IntView x_st, Int::IntView x_so, Int::IntView x_ot)
        : TernaryPropagator<Int::IntView, Int::PC_INT_DOM>(home, x_st,x_so,x_ot)
        , x_st(x_st)
        , x_so(x_so)
        , x_ot(x_ot)
    {
        x_st.subscribe(home, *this, Int::PC_INT_DOM);
        x_so.subscribe(home, *this, Int::PC_INT_DOM);
        x_ot.subscribe(home, *this, Int::PC_INT_DOM);
    }

    PathConsistency(Space& home, bool share, PathConsistency& p)
        : TernaryPropagator<Int::IntView, Int::PC_INT_DOM>(home, share,p)
    {
        x_st.update(home, share,p.x_st);
        x_so.update(home, share,p.x_so);
        x_ot.update(home, share,p.x_ot);
    }

    virtual PropCost cost(const Space&, const ModEventDelta&) const
    {
        return PropCost::ternary(PropCost::LO);
    }

    static ExecStatus post(Space& home, Int::IntView x_st, Int::IntView x_so, Int::IntView x_ot)
    {
        (void) new (home) PathConsistency(home,x_st,x_so,x_ot);
        return ES_OK;
    }

    /**
     * Remove values from views that ar in conflict with the constraint this
     * propagator implements
     *
     *  - must report whether failure occurred by returning an appropriate value
     *  for the execution status
     *  - must report when the propagator has become subsumed, that is, when the
     *  propagator can never again perform any propagation and should be
     *  disposed
     *  - characterise what it actually has computed: fixpoint foritself or not
     */
    virtual ExecStatus propagate(Space& home, const ModEventDelta&)
    {
        using namespace Int;
        bool assigned = x_st.assigned() && x_so.assigned() && x_ot.assigned();

//        bool assigned = false;
//        // x_st --> x_ot  intersect with (x_st--x_so composition with x_so--x_ot)
//        x_st
//
//        x_so.
//
//        composition(x_so,x_ot);
//        x_so
//        x_ot
        return assigned ? home.ES_SUBSUMED(*this) : ES_NOFIX;
    }

    //std::vector<int> composition(const std::vector<int>& i, const std::vector<int>& j)
    //{
    //    std::vector<int>::const_iterator i_it = i.begin();
    //    for(; i_it != i.end(); ++i_it)
    //    {
    //        std::vector<int>::const_iterator j_it = j.begin();
    //        for(; j_it != j_end(); ++j_it)
    //        {
    //            comp(*i_it,*j_it);
    //        }
    //    }
    //}

    std::vector<int> getValues(Int::IntView x)
    {
        std::vector<int> values;
        for(int i = 0; i < 2; ++i)
        {
            if(x.in(i))
            {
                values.push_back(i);
            }
        }
        return values;
    }

    virtual size_t dispose(Space& home) 
    {
        x_st.cancel(home, *this, Int::PC_INT_DOM);
        x_so.cancel(home, *this, Int::PC_INT_DOM);
        x_ot.cancel(home, *this, Int::PC_INT_DOM);
        (void) TernaryPropagator<Int::IntView, Int::PC_INT_DOM>::dispose(home);
        return sizeof(*this);
    }

    virtual Propagator* copy(Space& home, bool share)
    {
        return new (home) PathConsistency(home, share, *this);
    }

};

void path_consistent(Home home, IntVar x_st, IntVar x_so, IntVar x_ot)
{
    if(home.failed()) return;
    GECODE_ES_FAIL(PathConsistency::post(home,x_st,x_so,x_ot));
}

void pa_symmetric(Home home, IntVar x0, IntVar x1)
{
    if(home.failed()) 
    {
        LOG_WARN_S << "HOME failed for PASymmetry::post";
        return;
    }
    GECODE_ES_FAIL(PASymmetry::post(home,x0,x1));
}

}
#endif // TEMPL_SOLVERS_CSP_TEMPORAL_CSP_PROPAGATORS_HPP
