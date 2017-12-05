#ifndef TEMPL_SOLVERS_SESSION_HPP
#define TEMPL_SOLVERS_SESSION_HPP

#include "../Mission.hpp"
#include "Solution.hpp"

namespace templ {
namespace solvers {

class Session
{
public:
    typedef shared_ptr<Session> Ptr;

    Session();
    Session(const Mission::Ptr& mission);

    void setSolutions(const Solution::List& solutions) { mSolutions = solutions; }
    const Solution::List& getSolutions() const { return mSolutions; }

private:
    Mission::Ptr mpMission;
    Solution::List mSolutions;
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_SESSION_HPP
