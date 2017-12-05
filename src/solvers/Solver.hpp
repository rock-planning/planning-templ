#ifndef TEMPL_SOLVERS_SOLVER_HPP
#define TEMPL_SOLVERS_SOLVER_HPP

#include "../SharedPtr.hpp"
#include "../Configuration.hpp"
#include "Session.hpp"

namespace templ {
namespace solvers {

class Solver
{
public:
    typedef shared_ptr<Solver> Ptr;

    enum SolverType { UNKNOWN, CSP_TRANSPORT_NETWORK };

    static std::map<SolverType, std::string> SolverTypeTxt;

    virtual ~Solver();

    static Solver::Ptr getInstance(SolverType type);

    virtual Session::Ptr run(const Mission::Ptr& mission, uint32_t minNumberOfSolutions = 0, const Configuration& configuration = Configuration()) = 0;

protected:
    Solver();

    Solver(SolverType type);

    /**
     * Identify a starting solution, e.g., using heuristics
     */
    virtual Solution initialSolution();

    /**
     * Perform local search around a given seed solution
     */
    virtual Solution nextSolution(const Solution& seedSolution);

private:
    SolverType mSolverType;
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_SOLVER_HPP
