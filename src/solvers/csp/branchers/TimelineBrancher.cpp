#include "TimelineBrancher.hpp"
#include "../TransportNetwork.hpp"

namespace templ {
namespace solvers {
namespace csp {

size_t TimelineBrancher::getChoiceSize(Gecode::Space& space) const
{
    const TransportNetwork& network = static_cast<TransportNetwork&>(space);
    size_t numberOfFluents = network.getNumberOfFluents();
    size_t numberOfRoles = network.getActiveRoleList().size();

    // Upper bound on choice
    return numberOfFluents*numberOfFluents* numberOfRoles ;
}

bool TimelineBrancher::status(const Gecode::Space& home) const
{
    LOG_WARN_S << "STATUS OF TIMELINEBRANCHER with transportnetowrk"
        << static_cast<const TransportNetwork&>(home).toString();

    // Find the view with the fewest remaining unassigned views
    size_t timepoints = static_cast<const TransportNetwork&>(home).getNumberOfTimepoints();
    size_t fluents = static_cast<const TransportNetwork&>(home).getNumberOfFluents();

    // If we are currently not dealing with one particular waypoint
    if( timepoint == -1 || x[currentRole][ start[currentRole] ].assigned() )
    {
        size_t numberOfUnassignedViews = fluents*x.size();
        for(size_t t = 0; t < timepoints; ++t)
        {
            size_t unassigned = 0;
            for(size_t f = 0; f < fluents; ++f)
            {
                // pick new timepoint for status computation
                for(size_t i = 0; i < x.size(); ++i)
                {
                    const Gecode::Set::SetView& view = x[i][t*fluents + f];
                    if(!view.assigned())
                    {
                        ++unassigned;
                    }
                }
            }
            if(unassigned > 0 && unassigned < numberOfUnassignedViews)
            {
                timepoint = t;
                numberOfUnassignedViews = unassigned;
            }
            LOG_WARN_S << "TIMEPOINT: " << t << " with " << unassigned <<
                " unassigned views";
        }
        LOG_WARN_S << "SELECTED TIMEPOINT: " << timepoint << " with " << numberOfUnassignedViews <<
            " unassigned view";

        // reset start points -- since we have a new timeline view
        // we start at a particular timepoint
        for(size_t i = 0; i < x.size(); ++i)
        {
            start[i] = timepoint*fluents;
        }
    }

    // If timepoint remains at -1 -- a fully assigned multitimeline view has been computed
    if(timepoint == -1)
    {
        LOG_WARN_S << "Not timepoints remaining with unassigned views";
        return false;
    }

    // index of role to be used
    // Identify the corresponding timeline to choose from
    if( nextRoles.empty() )
    {
        LOG_WARN_S << "No roles in list -- repopulating";
        for(size_t i = 0; i < x.size(); ++i)
        {
            nextRoles.push_back(i);
        }
    }

    while(!nextRoles.empty())
    {
        size_t nextRoleIdx = rnd(nextRoles.size() - 1);
        size_t role = nextRoles[nextRoleIdx];

        LOG_WARN_S << "Brancher: " << id() << " trying role " << role;

        const TimelineView& timelineView = x[role];
        size_t end = (timepoint + 1)*fluents;
        for(size_t f = start[role]; f < end; ++f)
        {
            if(!timelineView[f].assigned())
            {
                currentRole = role;
                LOG_WARN_S << "Brancher: " << id() << " found unassigned: "
                    << " role: " << role << " fluent " << f << " " << timelineView[f];
                start[role] = f;

                // make sure the next time we use a different role first
                nextRoles.erase(nextRoles.begin() + nextRoleIdx);
                LOG_WARN_S << "Return status: true";
                return true;
            }
        }
        nextRoles.erase(nextRoles.begin() + nextRoleIdx);
    }

    // For this timepoint we did not find any particular open assignment
    timepoint = -1;
    LOG_WARN_S << "Return status: true";
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

    LOG_WARN_S << "Operation pre status " << view;
    LOG_WARN_S << " transportnetwork" << std::endl
        << static_cast<const TransportNetwork&>(home).toString();
    if(includeEmptySet && a == pv.alternatives() - 1)
    {
        LOG_WARN_S << "Brancher: " << id() << " COMMITING to choice: " << a << " role: " << role << " pos: " << pos << " val: "
            << "{}  -- in " << x.at(role)[pos];
        me = view.cardMax(home, 0);
    } else {
        LOG_WARN_S << "Choices available: " << choices.size();

        int val = choices[a];
        LOG_WARN_S << "Brancher: " << id() << " COMMITING to choice: " << a << " role: " << role << " pos: " << pos << " val: "
            << val << " -- in " << x.at(role)[pos];
        me = view.intersect(home, val, val);
    }

    if(Gecode::me_failed(me))
    {
        LOG_WARN_S << "Operation failed: result is" << view << " with status: " << me;
        LOG_WARN_S << " transportnetwork" << std::endl
            << static_cast<const TransportNetwork&>(home).toString();
        return Gecode::ES_FAILED;
    } else {
        LOG_WARN_S << "Operation success: result is" << view << " with status: " << me;
        LOG_WARN_S << " transportnetwork" << std::endl
            << static_cast<const TransportNetwork&>(home).toString();
        return Gecode::ES_OK;
    }
}

void branchTimelines(Gecode::Home home, const std::vector<Gecode::SetVarArray>& x)
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
    TimelineBrancher::post(home, timelinesView);
}


} // end namespace templ
} // end namespace solvers
} // end namespace csp
