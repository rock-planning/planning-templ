#include "Flaw.hpp"

using namespace graph_analysis::algorithms;

namespace templ {
namespace solvers {
namespace transshipment {

Flaw::Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
    const Role& role,
    SpaceTime::Point at
    )
    : violation(violation)
    , spacetime(at)
{
    affectedRoles.push_back(role);
}

Flaw::Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
    const Role::List& roles,
    SpaceTime::Point at
    )
    : violation(violation)
    , affectedRoles(roles)
    , spacetime(at)
{}

const Role& Flaw::affectedRole() const
{
    if(affectedRoles.empty())
    {
        throw std::runtime_error("templ::solvers::transshipment::Flaw::affectedRole: no affected roles availabled");
    } else if(affectedRoles.size() == 1)
    {
        return affectedRoles.front();
    } else {
        throw std::runtime_error("templ::solvers::transshipment::Flaw::affectedRole: multiple affected roles available");
    }
}

std::string Flaw::toString(size_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    switch(violation.getType())
    {
        case ConstraintViolation::MinFlow:
                ss << hspace << "Minflow violation in timeline:" << std::endl;
                break;
        case ConstraintViolation::TotalMinFlow:
                ss << hspace << "TotalMinflow violation in timeline:" << std::endl;
                break;
        case ConstraintViolation::TransFlow:
                ss << hspace << "Transflow violation in timeline:" << std::endl;
                break;
        case ConstraintViolation::TotalTransFlow:
                ss << hspace << "TotalTransflow violation in timeline:" << std::endl;
                break;

    }
    ss << violation.toString(indent + 8) << std::endl;
    ss << hspace << "    context:" << std::endl;
    ss << hspace << "    " << SpaceTime::toString(spacetime);
    return ss.str();
}

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
