#include "TimelineBrancher.hpp"
#include "../TransportNetwork.hpp"
#include <boost/numeric/conversion/cast.hpp>
#include <gecode/set/branch.hh>
#include <algorithm>
#include <random>
#include "SetNGL.hpp"

namespace templ {
namespace solvers {
namespace csp {


TimelineBrancher::PosVal::PosVal(const TimelineBrancher& b, int role, int p,
        std::vector<int> choices,
        int includeEmptySet)
    : Choice(b,choices.size() + includeEmptySet /* empty set */) // number of alternatives
    , role(role)
    , pos(p)
    , includeEmptySet(includeEmptySet)
    , choices(choices)
{
}

void TimelineBrancher::PosVal::archive(Gecode::Archive& e) const
{
    Gecode::Choice::archive(e);
    e << role << pos << includeEmptySet;
    for(const int& c : choices)
    {
        e << c;
    }
}

TimelineBrancher::TimelineBrancher(Gecode::Home home, MultiTimelineView& x0,
        const std::vector<int>& supplyDemand)
    : Gecode::Brancher(home)
    , x(x0)
    , mSupplyDemand(supplyDemand)
    , mRandom()
    , mRandomGenerator()
    , mStart(x0.size(), 0)
    , mCurrentRole(0)
    , mNextRoles()
    , mCurrentTimepoint(-1)
    , mAssignedRoles(x0.size(), -1)
{
    mRandom.hw();

    initialize(home);
}

TimelineBrancher::TimelineBrancher(Gecode::Space& space, TimelineBrancher& b)
    : Gecode::Brancher(space, b)
    , mSupplyDemand(b.mSupplyDemand)
    , mRandom(b.mRandom)
    , mRandomGenerator(b.mRandomGenerator)
    , mStart(b.mStart)
    , mCurrentRole(b.mCurrentRole)
    , mNextRoles(b.mNextRoles)
    , mCurrentTimepoint(b.mCurrentTimepoint)
    , mAssignedRoles(b.mAssignedRoles)
    , mChoiceSize(b.mChoiceSize)
    , mNumberOfTimepoints(b.mNumberOfTimepoints)
    , mNumberOfFluents(b.mNumberOfFluents)
    , mNumberOfRoles(b.mNumberOfRoles)
{
    for(size_t i = 0; i < b.x.size(); ++i)
    {
        TimelineView view;
        view.update(space, b.x[i]);
        x.push_back(view);
    }
    assert(x.size() == b.x.size());
}

void TimelineBrancher::initialize(Gecode::Space& space)
{
    const TransportNetwork& network = static_cast<TransportNetwork&>(space);
    mNumberOfTimepoints = network.getNumberOfTimepoints();
    mNumberOfFluents = network.getNumberOfFluents();
    mNumberOfRoles = network.getActiveRoleList().size();

    // Upper bound on choice
    // An edge can start from any fluent at timepoint t and
    // end at any fluent at timepoint t+1
    mChoiceSize = mNumberOfFluents*mNumberOfFluents* mNumberOfRoles ;
}

void TimelineBrancher::updateChoices(std::vector<int>& choices, Gecode::Set::SetView& view)
{
    // Identify possible values from the setview
    for(Gecode::SetVarLubRanges lub(view); lub(); ++lub)
    {
        for(int m = lub.min(); m <= lub.max(); ++m)
        {
            choices.push_back(m);
        }
    }
    // Identify assigned values
    for(Gecode::SetVarGlbValues glb(view); glb(); ++glb)
    {
        choices.push_back(glb.val());
    }

}

Gecode::Choice* TimelineBrancher::choice(Gecode::Space& home)
{
    // next best role activity
    int i = mStart[mCurrentRole];

    LOG_DEBUG_S << "CHOICE: role " << mCurrentRole << ", fluent: " << i << " val: " << x.at(mCurrentRole)[i].lubMax() << " -- val: " << x[mCurrentRole][i] <<  " assigned: " << x[mCurrentRole][i].assigned();

    // The view to be branched upon
    Gecode::Set::SetView& view = x[mCurrentRole][i];
    std::vector<int> choices;
    // update the list of choices, based on the given view
    updateChoices(choices, view);

    // if view is assigned, then it is either a single value or an empty set
    // Allow to pass an empty set for mSupplyDemand to neglect the supply demand constraints
    if(view.assigned() || mSupplyDemand.empty())
    {
        if(mSupplyDemand.empty())
        {
            LOG_WARN_S << "Ignoring capacity for generation of choices";
        }

        if(!choices.empty())
        {
            // Shuffle the choices
            // TODO: were should the randomization be best placed
            // Selection strategies etc.
            std::shuffle(choices.begin(), choices.end(), mRandomGenerator);

            return new PosVal(*this, mCurrentRole, i, choices, 0 /*includeEmptySet*/);
        } else {
            // When view is assigned, but there are no choices, then the set is empty
            return new PosVal(*this, mCurrentRole, i, choices, 1 /*includeEmptySet*/);
        }
    }

    // Only active when supply demand information is provided
    std::map<int, int> targetSupply;
    size_t fluents = static_cast<const TransportNetwork&>(home).getNumberOfFluents();

    int roleDemand = mSupplyDemand[mCurrentRole];
    if(!choices.empty())
    {
        // Find the best option -- so compute current target supply status first
        size_t end = (mCurrentTimepoint + 1)*fluents;
        for(size_t i = 0; i < boost::numeric_cast<size_t>(x.size()); ++i)
        {
            if(i == mCurrentRole)
            {
                continue;
            }

            // check where we have an assignment
            for(size_t f = mCurrentTimepoint*fluents; f < end; ++f)
            {
                const Gecode::Set::SetView& view = x[i][f];
                if(view.assigned() && view.lubSize() == 1)
                {
                    int target = view.lubMax();

                    if(!targetSupply.count(target))
                    {
                        targetSupply[target] = mSupplyDemand[i];
                    } else {
                        targetSupply[target] += mSupplyDemand[i];
                    }
                }
            }
        }
    }

    {
        std::stringstream ss;
        for(size_t i = 0; i < choices.size(); ++i)
        {
            ss << choices[i] << ", ";
        }
        LOG_DEBUG_S << "Number of choices (before) " << ss.str();
    }

    // Check our best alternatives -- if there is demand
    std::vector<int> bestChoices;
    for(size_t c = 0; c < choices.size(); ++c)
    {
        int choice = choices[c];
        LOG_INFO_S << "Target supply for choice: " << choice << " is: " << targetSupply[choice] << "(negative value represents demand)";

        // Local transition should always belong to the list of choices
        // no matter what
        if(i + fluents == boost::numeric_cast<size_t>(choice) )
        {
            bestChoices.push_back(choice);
            LOG_INFO_S << "Local transition is always best choice: " << choice;
            continue;
        }

        if(roleDemand < 0)
        {
            if( targetSupply[choice] > 0)
            {
                bestChoices.insert(bestChoices.begin(), choice);
                LOG_INFO_S << "Insert choice " << choice << " to benefit from existing transport";
                continue;
            }
        } else
        {
            // mobile system attracted by demand
            if( targetSupply[choice] < 0)
            {
                bestChoices.insert(bestChoices.begin(), choice);
                LOG_INFO_S << "Insert choice " << choice << " to support existing demand";
                continue;
            }
        }
    }
    if(bestChoices.empty())
    {
        //// TODO: use only usefull other choices, e.g.
        //// next hard commitments of particular roles --> cooperative approach
        //for(size_t i = 0; i < choices.size()*4/fluents; ++i)
        //{
        //    int targetIdx = mRandom(choices.size() -1);
        //    bestChoices.push_back(targetIdx);
        //    choices.erase(choices.begin() + targetIdx);
        //}
        bestChoices = choices;
    }

    {
        std::stringstream ss;
        for(size_t i = 0; i < bestChoices.size(); ++i)
        {
            ss << bestChoices[i] << ", ";
        }
        LOG_DEBUG_S << "Best choices for role " << mCurrentRole << ": "  << ss.str();
    }


    return new PosVal(*this, mCurrentRole, i, bestChoices, 0);
}

const Gecode::Choice* TimelineBrancher::choice(const Gecode::Space& home, Gecode::Archive& e)
{
    // Extracting choice from the given archive
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

Gecode::NGL* TimelineBrancher::ngl(Gecode::Space& home, const Gecode::Choice& c, unsigned int a) const
{
    const PosVal& pv = static_cast<const PosVal&>(c);

    // We focus the NGL on the (Not) empty branches which set the empty set
    // implicitly due to related path constraints
    if(pv.includeEmptySet && a == pv.alternatives()-1)
    {
        return NULL;
    } else {
        // Set branchers have IncNGL and ExcNGL available by default
        //return new (home) Gecode::Set::Branch::ExcNGL(home, x[ pv.role ] [ pv.pos ], pv.choices[a]);
        // initialize by ExcNGL( space, SetView and int
        LOG_DEBUG_S << "TimelineBrancher: disallow for role " << pv.role << " and pos: " << pv.pos <<" particular value for setview: " << pv.choices[a];
        return new (home) ExcNGL(home, x[ pv.role ] [ pv.pos ], pv.choices[a]);
    }
}

void TimelineBrancher::post(Gecode::Home home, MultiTimelineView& x, const std::vector<int> mSupplyDemand)
{
    (void) new (home) TimelineBrancher(home, x, mSupplyDemand);
}

bool TimelineBrancher::status(const Gecode::Space& home) const
{
    LOG_DEBUG_S << "Status of timelinebrancher with transportnetowrk"
        << static_cast<const TransportNetwork&>(home).toString();

    // ------------------------------------------------------------
    // (A) Find the view with the fewest remaining unassigned views for the
    // mCurrentRole
    //
    // If we are currently not dealing with one particular waypoint
    // meaning no particular timepoint and no assigned value
    if( mCurrentTimepoint == -1 || x[mCurrentRole][ mStart[mCurrentRole] ].assigned() )
    {
        // initialize with maximum number of unassigned views
        size_t numberOfUnassignedViews = mNumberOfFluents*x.size();
        // initialize with the maximum number of roles
        size_t numberOfAssignedWaypoints = x.size();

        size_t preferredWaypointByUnassignedViews = mNumberOfTimepoints;
        size_t preferredWaypointByAssignedWaypoints = mNumberOfTimepoints;
        for(size_t t = 0; t < mNumberOfTimepoints; ++t)
        {
            size_t unassigned = 0;
            size_t assignedWaypoints = 0;
            for(size_t f = 0; f < mNumberOfFluents; ++f)
            {
                // pick new timepoint for status computation
                for(size_t i = 0; i < x.size(); ++i)
                {
                    const Gecode::Set::SetView& view = x[i][t*mNumberOfFluents + f];
                    if(!view.assigned())
                    {
                        ++unassigned;
                    } else if(view.cardMin() == 1) // if set is not empty
                    {
                        ++assignedWaypoints;
                    }
                }
            }

            // cache timepoint with respect to fewest unassigned views
            if(unassigned > 0 && unassigned < numberOfUnassignedViews)
            {
                preferredWaypointByUnassignedViews = t;
                numberOfUnassignedViews = unassigned;
            }
            // cache timepoint with respect to fewest assigned views
            if(assignedWaypoints > 0 && assignedWaypoints < numberOfAssignedWaypoints)
            {
                preferredWaypointByAssignedWaypoints = t;
                numberOfAssignedWaypoints = assignedWaypoints;
            }
            LOG_DEBUG_S << "TIMEPOINT: " << t << " with:" << std::endl
                << "    " << unassigned << " unassigned views"
                << "    " << assignedWaypoints << " assigned waypoints";
        }

        // Select the timepoint with fewest assigned waypoints first,
        // then the timepoint with fewest unassigned views
        if(preferredWaypointByAssignedWaypoints != mNumberOfTimepoints)
        {
            mCurrentTimepoint = preferredWaypointByAssignedWaypoints;
            LOG_DEBUG_S << "Timepoint selection using assigned waypoints preference";
        } else if (preferredWaypointByUnassignedViews != mNumberOfTimepoints)
        {
            mCurrentTimepoint = preferredWaypointByUnassignedViews;
            LOG_DEBUG_S << "Timepoint selection using unassigned view preference";
        }
        LOG_DEBUG_S << "SELECTED TIMEPOINT: " << mCurrentTimepoint << " with " << numberOfUnassignedViews <<
            " unassigned view";

        // reset mStart points -- since we have a new timeline view
        // we mStart at a particular timepoint
        for(size_t i = 0; i < x.size(); ++i)
        {
            mStart[i] = mCurrentTimepoint*mNumberOfFluents;
        }
    }

    // If timepoint remains at -1 -- no unassigned view has been found
    // and thus a fully assigned multitimeline view has been computed
    if(mCurrentTimepoint == -1)
    {
        MultiTimelineView::const_iterator cit = x.begin();
        assert(!x.empty());
        for(; cit != x.end(); ++cit)
        {
            if(!cit->assigned())
            {
                LOG_DEBUG_S << "Timeline with unassigned views";
                return true;
            } else {
                LOG_DEBUG_S << "Timeline assigned: " << *cit;
            }
        }
        LOG_DEBUG_S << "All timepoints with assigned views";
        return false;
    }

    // -------------------------------------------------------------
    // (B) Identify the corresponding timeline to choose from
    // , i.e. identify first index of role to be used
    if( mNextRoles.empty() )
    {
        LOG_DEBUG_S << "No roles in list -- repopulating";
        for(size_t i = 0; i < x.size(); ++i)
        {
            mNextRoles.push_back(i);
        }
    }

    while(!mNextRoles.empty())
    {
        // Pick role randomly from the remaining set of roles
        size_t nextRoleIdx = mRandom(static_cast<unsigned long long>(mNextRoles.size() - 1));
        size_t role = mNextRoles[nextRoleIdx];

        LOG_DEBUG_S << "Brancher: " << id() << " trying role " << role;

        // Pick view corresponding to randomly chosen role
        // and mark the end range
        const TimelineView& timelineView = x[role];
        size_t end = (mCurrentTimepoint + 1)*mNumberOfFluents;
        for(size_t f = mStart[role]; f < end; ++f)
        {
            // pick first unassigned view and set it as new
            // mStart value
            if(!timelineView[f].assigned())
            {
                mCurrentRole = role;
                LOG_DEBUG_S << "Brancher: " << id() << " found unassigned: "
                    << " role: " << role << " fluent " << f << " " << timelineView[f];
                mStart[role] = f;

                // make sure the next time we use a different role first
                mNextRoles.erase(mNextRoles.begin() + nextRoleIdx);
                LOG_DEBUG_S << "Return status: true";
                // Return true if there are unassigned views left
                return true;
            }
        }
        mNextRoles.erase(mNextRoles.begin() + nextRoleIdx);
    }

    // For this timepoint we did not find any particular open assignment
    // so trigger
    mCurrentTimepoint = -1;
    LOG_DEBUG_S << "Return status: true";
    // Return true if there are unassigned views left
    return true;
}

Gecode::ExecStatus TimelineBrancher::commit(Gecode::Space& home,
        const Gecode::Choice& c,
        unsigned int a)
{
    const PosVal& pv = static_cast<const PosVal&>(c);
    int role = pv.role;
    int pos = pv.pos;
    int includeEmptySet = pv.includeEmptySet;

    std::vector<int> choices = pv.choices;

    Gecode::Set::SetView& view = x[role][pos];
    Gecode::ModEvent me;

    LOG_DEBUG_S << "Operation pre status " << view;
    LOG_DEBUG_S << " transportnetwork" << std::endl
        << static_cast<const TransportNetwork&>(home).toString();
    if(includeEmptySet && a == pv.alternatives() - 1)
    {
        LOG_DEBUG_S << "Brancher: " << id() << " COMMITING to choice: " << a << " role: " << role << " pos: " << pos << " val: "
            << "{}  -- in " << x.at(role)[pos];
        me = view.cardMax(home, 0);
    } else {
        LOG_DEBUG_S << "Choices available: " << choices.size();

        int val = choices[a];
        LOG_DEBUG_S << "Brancher: " << id() << " COMMITING to choice: " << a << " role: " << role << " pos: " << pos << " val: "
            << val << " -- in " << x.at(role)[pos];
        me = view.intersect(home, val, val);
    }

    if(Gecode::me_failed(me))
    {
        LOG_INFO_S << "Operation failed: result is" << view << " with status: " << me;
        LOG_INFO_S << " transportnetwork" << std::endl
            << static_cast<const TransportNetwork&>(home).toString();
        return Gecode::ES_FAILED;
    } else {
        LOG_INFO_S << "Operation success: result is" << view << " with status: " << me;
        return Gecode::ES_OK;
    }
}

void TimelineBrancher::print(const Gecode::Space& home,
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
Gecode::Actor* TimelineBrancher::copy(Gecode::Space& home)
{
    return new (home) TimelineBrancher(home, *this);
}

size_t TimelineBrancher::dispose(Gecode::Space& home)
{
    //home.ignore(*this, Gecode::AP_DISPOSE);
    (void) Gecode::Brancher::dispose(home);
    return sizeof(*this);
}


void branchTimelines(Gecode::Home home, const std::vector<Gecode::SetVarArray>& x,
        const std::vector<int>& mSupplyDemand)
{
    if(home.failed())
    {
        return;
    }
    TimelineBrancher::MultiTimelineView timelinesView;
    for(size_t i = 0; i < x.size(); ++i)
    {
        TimelineBrancher::TimelineView y(home, Gecode::SetVarArgs(x[i]));
        timelinesView.push_back(y);
    }
    TimelineBrancher::post(home, timelinesView, mSupplyDemand);
}


} // end namespace templ
} // end namespace solvers
} // end namespace csp
