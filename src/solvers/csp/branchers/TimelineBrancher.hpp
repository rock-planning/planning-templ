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
    typedef std::vector<TimelineView> MultiTimelineView;
    MultiTimelineView x;
    // supply demand per Role
    std::vector<int> supplyDemand;

    mutable Gecode::Rnd rnd;

    // The brancher has to maintain the invariant that all x_i are
    // assigned for 0 < i < start (see also Gecode MPG Section 31.2.2)
    mutable std::vector<int> start;
    mutable size_t currentRole;
    mutable std::vector<size_t> nextRoles;

    mutable int timepoint;
    // of this particular timepoint
    mutable std::vector<int> assignedRoles;

    class PosVal : public Gecode::Choice
    {
    public:
        int role;
        int pos;
        int includeEmptySet;
        std::vector<int> choices;

        PosVal(const TimelineBrancher& b, int role, int p,
                std::vector<int> choices,
                int includeEmptySet)
            : Choice(b,choices.size() + includeEmptySet /* empty set */) // number of alternatives
            , role(role)
            , pos(p)
            , includeEmptySet(includeEmptySet)
            , choices(choices)
        {
            LOG_WARN_S << "Created pos val with " << choices.size() << " choices";
        }

        virtual size_t size(void) const
        {
            return sizeof(*this);
        }

        virtual void archive(Gecode::Archive& e) const
        {
            Gecode::Choice::archive(e);
            e << role << pos << includeEmptySet;
            for(const auto& c : choices)
            {
                LOG_WARN_S << "Packing choice into archive";
                e << c;
            }
        }

    };

    TimelineBrancher(Gecode::Home home, MultiTimelineView& x0,
            const std::vector<int>& supplyDemand)
        : Gecode::Brancher(home)
        , x(x0)
        , supplyDemand(supplyDemand)
        , rnd()
        , start(x0.size(), 0)
        , currentRole(0)
        , nextRoles()
        , timepoint(-1)
        , assignedRoles(x0.size(), -1)
    {
        rnd.time();
    }

    size_t getChoiceSize(Gecode::Space& space) const;

    TimelineBrancher(Gecode::Space& space, bool share, TimelineBrancher& b)
        : Gecode::Brancher(space, share, b)
        , supplyDemand(b.supplyDemand)
        , rnd(b.rnd)
        , start(b.start)
        , currentRole(b.currentRole)
        , nextRoles(b.nextRoles)
        , timepoint(b.timepoint)
        , assignedRoles(b.assignedRoles)
    {
        for(size_t i = 0; i < b.x.size(); ++i)
        {
            TimelineView view;
            view.update(space, share, b.x[i]);
            x.push_back(view);
        }
    }

    static void post(Gecode::Home home, MultiTimelineView& x, const std::vector<int> supplyDemand)
    {
        (void) new (home) TimelineBrancher(home, x, supplyDemand);
    }

    /**
     * Return true if alternatives are left
     *
     * Checks all timelines if there are unassigned views
     */
    virtual bool status(const Gecode::Space& home) const;

    virtual Gecode::Choice* choice(Gecode::Space& home);

    /**
     * Transform Gecode::Archive to the corresponding choice
     */
    virtual const Gecode::Choice* choice(const Gecode::Space& home, Gecode::Archive& e)
    {
        int role = e[0];
        int pos = e[1];
        int includeEmptySet = e[2];
        std::vector<int> choices;
        for(int i = 3; i < e.size(); ++i)
        {
            choices.push_back(e[i]);
        }
        return new PosVal(*this, role, pos, choices, includeEmptySet);
    }

    /**
     * Perform commit for choice \a _c  and alternative \a
     */
    virtual Gecode::ExecStatus commit(Gecode::Space& home,
            const Gecode::Choice& c,
            unsigned int a);

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

        // Last alternatives
        if(pv.includeEmptySet && a == pv.alternatives() - 1)
        {
            o << "x[" << pos << "] = {}";
        } else
        {
            o << "x[" << pos << "] = { " << pv.choices[a] << "}";
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

void branchTimelines(Gecode::Home home, const std::vector<Gecode::SetVarArray>& x,
        const std::vector<int>& supplyDemand);

} // end namespace templ
} // end namespace solvers
} // end namespace csp
#endif // TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP
