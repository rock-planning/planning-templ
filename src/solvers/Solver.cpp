#include "Solver.hpp"
#include "csp/TransportNetwork.hpp"

namespace templ {
namespace solvers {

std::map<Solver::SolverType, std::string> Solver::SolverTypeTxt = {
    { Solver::UNKNOWN, "unknown" },
    { Solver::CSP_TRANSPORT_NETWORK, "csp_transport_network" }
};

Solver::Solver()
    : mSolverType(UNKNOWN)
{}

Solver::Solver(SolverType type)
    : mSolverType(type)
{}

Solver::~Solver()
{}

Solver::Ptr Solver::getInstance(SolverType type)
{
    switch(type)
    {
        case CSP_TRANSPORT_NETWORK:
            return make_shared<csp::TransportNetwork>();
        default:
            throw std::invalid_argument("templ::solvers::Solver::getInstance: "
                    "unknown solver type requested");
    }
}

Solution Solver::construct(const Mission::Ptr& mission,
        StoppingCriteria c)
{
    Solution solution;
    return solution;
}

Solution Solver::nextSolution(const Solution& seedSolution,
        StoppingCriteria c)
{
    return seedSolution;
}

} // end namespace solvers
} // end namespace templ
