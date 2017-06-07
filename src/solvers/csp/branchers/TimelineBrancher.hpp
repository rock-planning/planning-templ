#ifndef TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP
#define TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP

#include "../Types.hpp"
#include <base-logging/Logging.hpp>
#include <gecode/set/branch.hh>
#include "SetNGL.hpp"

namespace templ {
namespace solvers {
namespace csp {

/**
 * Timeline Brancher will allow us to perform a guided local search approach
 * The timeline brancher branches on a set of timelines
 *
 * The main restriction is:
 * - a transition can only go from t to t+1 within one timeline
 */
class TimelineBrancher : public Gecode::Brancher
{
public:
    /// The view array for a single timeline
    typedef Gecode::ViewArray<Gecode::Set::SetView> TimelineView;
    /// The list of multiple timelines
    typedef std::vector<TimelineView> MultiTimelineView;

    /// The views to branch upon
    MultiTimelineView x;

    /// Supply demand per Role
    std::vector<int> supplyDemand;

    mutable Gecode::Rnd rnd;
    mutable std::default_random_engine randomGenerator;

    // The brancher has to maintain the invariant that all x_i are
    // assigned for 0 < i < start (see also Gecode MPG Section 31.2.2)
    // The start is maintained per individual timeline
    mutable std::vector<int> start;

    /// The current role (and thus timeline) the brancher operates on
    mutable size_t currentRole;
    //// The next roles that have to be handled
    mutable std::vector<size_t> nextRoles;

    /// The current timepoint
    mutable int timepoint;
    /// of this particular timepoint
    mutable std::vector<int> assignedRoles;

    /**
     * A PosVal represents the Choice for the brancher and
     * refers to a particular role (to identify the timeline)
     * and a position in this timeline
     */
    class PosVal : public Gecode::Choice
    {
    public:
        int role;
        int pos;
        int includeEmptySet;
        std::vector<int> choices;

        PosVal(const TimelineBrancher& b, int role, int p,
                std::vector<int> choices,
                int includeEmptySet);

        virtual size_t size(void) const
        {
            return sizeof(*this);
        }

        virtual void archive(Gecode::Archive& e) const;
    };

    TimelineBrancher(Gecode::Home home, MultiTimelineView& x0,
            const std::vector<int>& supplyDemand);

    TimelineBrancher(Gecode::Space& space, bool share, TimelineBrancher& b);

    size_t getChoiceSize(Gecode::Space& space) const;

    /**
     * Update the list of choices by extraction of feasible values from a given SetView
     */
    void updateChoices(std::vector<int>& choices, Gecode::Set::SetView& view);

    static void post(Gecode::Home home, MultiTimelineView& x, const std::vector<int> supplyDemand);

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
    virtual const Gecode::Choice* choice(const Gecode::Space& home, Gecode::Archive& e);

    /**
     * Perform commit for choice \a _c  and alternative \a
     */
    virtual Gecode::ExecStatus commit(Gecode::Space& home,
            const Gecode::Choice& c,
            unsigned int a);

    /**
     * Create a no-nood literal for choice c and alternative a
     */
    virtual Gecode::NGL* ngl(Gecode::Space& home, const Gecode::Choice& e, unsigned int a) const;

    /**
     * Print an explanation
     */
    virtual void print(const Gecode::Space& home,
            const Gecode::Choice& c,
            unsigned int a,
            std::ostream& o) const;

    virtual Gecode::Actor* copy(Gecode::Space& home, bool share);
    virtual size_t dispose(Gecode::Space& home);

};

void branchTimelines(Gecode::Home home, const std::vector<Gecode::SetVarArray>& x,
        const std::vector<int>& supplyDemand);

} // end namespace templ
} // end namespace solvers
} // end namespace csp
#endif // TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP
