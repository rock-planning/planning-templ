#ifndef TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP
#define TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP

#include "../Mission.hpp"
#include "../SpaceTime.hpp"
#include "Solution.hpp"

namespace templ {
namespace solvers {

/**
 * Allow to analyse an existing solution upon quality, cost, and
 * redundancy
 */
class SolutionAnalysis
{
public:
    SolutionAnalysis(const Mission::Ptr& mission, const SpaceTime::Network& solution);

    void analyse();

    double getQuality() const { return mQuality; }
    double getCost() const { return mCost; }
    double getRedundancy() const { return mRedundancy; }

    /**
     * Retrieve the list of required roles / all roles that are involved in this
     * problem
     *
     * A required role is given, it its usage exceeds 1 (since the initial usage
     * is given by its availability at the initial position)
     */
    std::set<Role> getRequiredRoles(size_t minRequirement = 1) const;

    /**
     * Get the model availability over the course of one interval
     */
    std::vector<organization_model::ModelPool> getAvailableResources(const symbols::constants::Location::Ptr& location, const solvers::temporal::Interval& interval) const;

//
//    ModelPool getAvailableFunctionalities(const symbols::constants::Location::Ptr& location, const solvers::temporal::Interval& interval) const;
//
//    std::vector< SpaceTime::Point > getTimeline(const IRI& role);

//    bool hasAvailableFunctionality(const SpaceTime::Point& spaceTimePoint, const IRI& functionality) const;
//    bool hasAvailableResource(const SpaceTime::Point& spaceTimePoint, const IRI& resource) const;
//
//    double getFunctionalityRedundancy(const SpaceTime::Point& spaceTimePoint, const IRI& functionality) const;
//    double getResourceRedundancy(const SpaceTime::Point& spaceTimePoint, const IRI& resource) const;

    // getTransfer

    graph_analysis::BaseGraph::Ptr toHyperGraph();

private:
    /**
     * Check solution with respect to the given requirement
     */
    void analyse(const solvers::csp::FluentTimeResource& requirement);

    double degreeOfFulfillment(const solvers::csp::FluentTimeResource& requirement);


    Mission::Ptr mpMission;
    SpaceTime::Network mSolutionNetwork;

    double mQuality;
    double mCost;
    double mRedundancy;

};

} // end namespace solvers
} // end namespace templ
#endif // TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP
