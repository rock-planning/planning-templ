#ifndef TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP
#define TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP

#include <map>
#include <moreorg/vocabularies/Robot.hpp>
#include "../FluentTimeResource.hpp"
#include "../../SpaceTime.hpp"
#include "../temporal/point_algebra/TimePointComparator.hpp"

namespace templ {
namespace solvers {
namespace csp {

/**
 * A conveniance class to map a role to its respective timeline
 * There are initially no assumptions on the correct order of
 * timepoint in the timeline
 */
class RoleTimeline
{
public:
    RoleTimeline();

    RoleTimeline(const Role& role,
            const moreorg::OrganizationModelAsk& ask);

    void setRole(const Role& role) { mRole = role; }

    const Role& getRole() const { return mRole; }

    const moreorg::facades::Robot& getRobot() const { return mRobot; }

    /**
     * Set the actual timeline
     */
    void setTimeline(const SpaceTime::Timeline& timeline) { mTimeline = timeline; }

    /**
     * Get the timeline
     * \return timeline
     */
    const SpaceTime::Timeline& getTimeline() const { return mTimeline; }


    void add(const SpaceTime::Point& point) { mTimeline.push_back(point); }

    bool operator<(const RoleTimeline& other) const;

    size_t size() const { return mTimeline.size(); }

    /**
     * Sort this timeline according to time
     * \param tcp TimePointComparator that shall be used for sorting of the
     * timeline
     */
    void sort(const temporal::point_algebra::TimePointComparator& tpc);

    /**
     * Stringify role timeline
     */
    std::string toString(size_t indent = 0, bool withStats = true) const;

    /**
     * Convert a role->timeline map to a string representation
     * \return timeline representation
     */
    static std::string toString(const std::map<Role, RoleTimeline>& timelines,
            uint32_t indent = 0, bool withStats = true);

    /**
     * Collect all space time timelines
     */
    static SpaceTime::Timelines collectTimelines(const std::map<Role, RoleTimeline>& timelines);

    /**
     * Compute the travel distance based on the generated path from the timeline
     * Note, that this obviously depends on the correct temporal ordering of the
     * timepoints, which has to be accounted beforehand
     * \return distance in m
     */
    double travelDistance() const;

    /**
     * Estimate the energy cost based on the travel distance and estimated
     * duration for this role model
     * \see duration
     * \see travelDistance
     * \return cost in Wh
     */
    double estimatedEnergyCost() const;

    /**
     * Duration of the timeline estimated from the travel distance and
     * the role model's properties
     * \return duration in second
     */
    double duration() const;
private:
    Role mRole;
    moreorg::OrganizationModelAsk mAsk;
    SpaceTime::Timeline mTimeline;

    moreorg::facades::Robot mRobot;
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_ROLE_TIMELINE_HPP
