#ifndef TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP
#define TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP

#include "../Types.hpp"
#include <base-logging/Logging.hpp>
#include <gecode/set/branch.hh>
#include <random>

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

    /**
     * A PosVal represents the Choice for the brancher and
     * refers to a particular role (to identify the timeline)
     * and a position in this timeline
     *
     * Additionally is has a flag to tell whether the empty set should be
     * included or not
     */
    class PosVal : public Gecode::Choice
    {
    public:
        /// Role idx to identify the timeline
        int role;
        /// Position to identify the position in the timeline
        int pos;
        /// Flag to define if the empty set is part of the chocies or not
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
            const std::vector<int>& mSupplyDemand);

    TimelineBrancher(Gecode::Space& space, TimelineBrancher& b);

    void initialize(Gecode::Space& space);

    size_t getChoiceSize(Gecode::Space& space) const { return mChoiceSize; }

    /**
     * Update the list of choices by extraction of feasible values from a given SetView
     * \param choices list of choices to update
     * \param view SetView to extract feasible values from
     */
    void updateChoices(std::vector<int>& choices, Gecode::Set::SetView& view);

    static void post(Gecode::Home home, MultiTimelineView& x, const std::vector<int> mSupplyDemand);

    /**
     * Checks all timelines if there are unassigned views
     * \return true if alternatives are left
     */
    virtual bool status(const Gecode::Space& home) const;

    /**
     * Create a choice
     */
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

    virtual Gecode::Actor* copy(Gecode::Space& home);
    virtual size_t dispose(Gecode::Space& home);

private:
    /// The views to branch upon
    MultiTimelineView x;

    /// Supply demand per Role
    std::vector<int> mSupplyDemand;

    mutable Gecode::Rnd mRandom;
    mutable std::default_random_engine mRandomGenerator;

    // The brancher has to maintain the invariant that all x_i are
    // assigned for 0 < i < start (see also Gecode MPG Section 31.2.2)
    // The mStart is maintained per individual timeline
    mutable std::vector<int> mStart;

    /// The current role (and thus timeline) the brancher operates on
    mutable size_t mCurrentRole;
    //// The next roles that have to be handled
    mutable std::vector<size_t> mNextRoles;

    /// The current timepoint
    mutable int mCurrentTimepoint;
    /// of this particular timepoint
    mutable std::vector<int> mAssignedRoles;

    size_t mChoiceSize;
    size_t mNumberOfTimepoints;
    size_t mNumberOfFluents;
    size_t mNumberOfRoles;


};

void branchTimelines(Gecode::Home home, const std::vector<Gecode::SetVarArray>& x,
        const std::vector<int>& mSupplyDemand);

} // end namespace templ
} // end namespace solvers
} // end namespace csp
#endif // TEMPL_SOLVERS_CSP_BRANCHERS_TIMELINE_BRANCHER_HPP
