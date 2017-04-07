#ifndef TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP
#define TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP

#include "../Types.hpp"
#include <base-logging/Logging.hpp>

namespace templ {
namespace solvers {
namespace csp {

/**
 * Timeline Brancher will allow us to perform a guided local search approach
 *
 */
class TimelineBrancher : public Gecode::Brancher
{
public:
    typedef Gecode::ViewArray<Gecode::Set::SetView> TimelineView;
    TimelineView x;

    // The brancher has to maintain the invariant that all x_i are
    // assigned for 0 < i < start (see also Gecode MPG Section 31.2.2)
    mutable int start;
    size_t choiceSize;

    class PosVal : public Gecode::Choice
    {
    public:
        int pos;
        int val;

        PosVal(const TimelineBrancher& b, int p, int v)
            : Choice(b, 2) // number of alternatives
            , pos(p)
            , val(v)
        {
        }

        virtual size_t size(void) const
        {
            return sizeof(*this);
        }

        virtual void archive(Gecode::Archive& e) const
        {
            Gecode::Choice::archive(e);
            e << pos << val;
        }

    };

    TimelineBrancher(Gecode::Home home, TimelineView& x0)
        : Gecode::Brancher(home)
        , x(x0)
        , start(0)
        //, choiceSize( getChoiceSize(home) )
    {}

    size_t getChoiceSize(Gecode::Space& space) const;

    TimelineBrancher(Gecode::Space& space, bool share, TimelineBrancher& b)
        : Gecode::Brancher(space, share, b)
        , start(b.start)
        , choiceSize(b.choiceSize)
    {
        x.update(space, share, b.x);
    }

    static void post(Gecode::Home home, TimelineView& x)
    {
        (void) new (home) TimelineBrancher(home, x);
    }

    // Return true if alternatives are left
    virtual bool status(const Gecode::Space& home) const
    {
        for(int i = start; i < x.size(); ++i)
        {
            if(!x[i].assigned())
            {
                start = i;
                return true;
            }
        }
        return false;
    }

    virtual Gecode::Choice* choice(Gecode::Space& home)
    {
        return new PosVal(*this, start, x[start].lubMax());
    }

    /**
     * Transform Gecode::Archive to the corresponding choice
     */
    virtual const Gecode::Choice* choice(const Gecode::Space& home, Gecode::Archive& e)
    {
        int pos, val;
        e >> pos >> val;
        return new PosVal(*this, pos, val);
    }

    /**
     * Perform commit for choice \a _c  and alternative \a
     */
    virtual Gecode::ExecStatus commit(Gecode::Space& home,
            const Gecode::Choice& c,
            unsigned int a)
    {
        const PosVal& pv = static_cast<const PosVal&>(c);
        int pos = pv.pos;
        int val = pv.val;

        LOG_WARN_S << "COMMITING to choice: " << a << " " << x[pos];
        if(a == 0)
        {
            return Gecode::me_failed( x[pos].include(home, val) ) ? Gecode::ES_FAILED : Gecode::ES_OK;
        } else {
            return Gecode::me_failed( x[pos].exclude(home, val) ) ? Gecode::ES_FAILED : Gecode::ES_OK;
        }
    }

    /**
     * Print an explanation
     */
    virtual void print(const Gecode::Space& home,
            const Gecode::Choice& c,
            unsigned int a,
            std::ostream& o) const
    {
        const PosVal& pv = static_cast<const PosVal&>(c);
        int pos = pv.pos;
        int val = pv.val;

        if(a == 0)
        {
            o << "x[" << pos << "] = " << val;
        } else {
            o << "x[" << pos << "] != " << val;
        }
    }

    virtual Gecode::Actor* copy(Gecode::Space& home, bool share)
    {
        return new (home) TimelineBrancher(home, share, *this);
    }

    virtual size_t dispose(Gecode::Space& home)
    {
        //home.ignore(*this, Gecode::AP_DISPOSE);
        (void) Gecode::Brancher::dispose(home);
        return sizeof(*this);
    }
};

void branchTimelines(Gecode::Home home, const Gecode::SetVarArgs& x);

} // end namespace templ
} // end namespace solvers
} // end namespace csp
#endif // TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP
