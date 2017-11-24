#include "SimpleConstraint.hpp"
#include <sstream>
#include <iostream>

namespace templ {
namespace constraints {

SimpleConstraint::SimpleConstraint()
    : Constraint()
    , Edge()
{}

SimpleConstraint::SimpleConstraint(Category category)
    : Constraint(category)
    , Edge()
{}

SimpleConstraint::SimpleConstraint(Category category, const Variable::Ptr& source, const Variable::Ptr& target)
    : Constraint(category)
    , Edge(source, target)
{}

SimpleConstraint::~SimpleConstraint()
{}

std::string SimpleConstraint::toString() const
{
    return toString(0);
}

std::string SimpleConstraint::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "SimpleConstraint" << std::endl;
    ss << getSourceVertex()->toString(indent + 4) << std::endl;
    ss << getTargetVertex()->toString(indent + 4) << std::endl;
    return ss.str();
}

} // end namespace constraints
} // end namespace templ
