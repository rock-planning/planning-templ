#ifndef TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP
#define TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP

#include "../Mission.hpp"
#include "Solution.hpp"

namespace templ {
namespace solvers {

class SolutionAnalysis
{
public:
    SolutionAnalysis(const Mission::Ptr& mission, const Solution& solution);

    void analyse();

    double getQuality() const { return mQuality; }
    double getCost() const { return mCost; }
    double getRedundancy() const { return mRedundancy; }


private:
    /**
     * Check solution with respect to the given requirement
     */
    void analyse(const solvers::csp::FluentTimeResource& requirement);



    Mission::Ptr mpMission;
    Solution mSolution;

    double mQuality;
    double mCost;
    double mRedundancy;

};

} // end namespace solvers
} // end namespace templ
#endif // TEMP_SOLVERS_SOLUTION_ANALYSIS_HPP
