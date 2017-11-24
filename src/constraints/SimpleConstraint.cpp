#include "SimpleConstraint.hpp"
#include <sstream>

namespace templ {
namespace constraints {

SimpleConstraint::SimpleConstraint(Type type)
    : Constraint(type)
    , Edge()
{}

SimpleConstraint::~SimpleConstraint()
{}


SimpleConstraint::SimpleConstraint(Type type, const Variable::Ptr& source, const Variable::Ptr& target)
    : Constraint(type)
    , Edge(source, target)
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
