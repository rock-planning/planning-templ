#include "Flaw.hpp"

using namespace graph_analysis::algorithms;

namespace templ {
namespace solvers {
namespace transshipment {

Flaw::Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
    const Role& role,
    SpaceTime::Point at
    )
    : mViolation(violation)
    , mFromSpacetime(at)
{
    mAffectedRoles.push_back(role);
}

Flaw::Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
    const Role::List& roles,
    SpaceTime::Point at
    )
    : mViolation(violation)
    , mAffectedRoles(roles)
    , mFromSpacetime(at)
{}

Flaw::Flaw(const graph_analysis::algorithms::ConstraintViolation& violation,
    const Role::List& roles,
    SpaceTime::Point from,
    SpaceTime::Point to
    )
    : mViolation(violation)
    , mAffectedRoles(roles)
    , mFromSpacetime(from)
    , mToSpacetime(to)
{}

const Role& Flaw::affectedRole() const
{
    if(mAffectedRoles.empty())
    {
        throw std::runtime_error("templ::solvers::transshipment::Flaw::affectedRole: no affected roles availabled");
    } else if(mAffectedRoles.size() == 1)
    {
        return mAffectedRoles.front();
    } else {
        throw std::runtime_error("templ::solvers::transshipment::Flaw::affectedRole: multiple affected roles available");
    }
}

std::string Flaw::toString(size_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    switch(mViolation.getType())
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
    ss << mViolation.toString(indent + 8) << std::endl;
    ss << hspace << "    context:" << std::endl;
    if(isVertexFlaw())
    {
        ss << hspace << "    at:" << SpaceTime::toString(mFromSpacetime);
    } else {
        ss << hspace << "    from:" << SpaceTime::toString(mFromSpacetime);
        ss << hspace << "    to:" << SpaceTime::toString(mToSpacetime);
    }
    return ss.str();
}

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
