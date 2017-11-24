#include "HyperConstraint.hpp"
#include <sstream>

namespace templ {
namespace constraints {

HyperConstraint::HyperConstraint(Type type)
    : Constraint(type)
{}

HyperConstraint::HyperConstraint(Type type, const Variable::PtrList& sources,
        const Variable::PtrList& targets)
    : Constraint(type)
    , DirectedHyperEdge(sources, targets)
{}

HyperConstraint::~HyperConstraint()
{}

std::string HyperConstraint::toString() const
{
    return toString(0);
}

std::string HyperConstraint::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "HyperConstraint:" << std::endl;
    ss << hspace << "    from:" << std::endl;
    for(const Vertex::Ptr& from : getSourceVertices())
    {
        ss << from->toString(indent + 8) << std::endl;
    }
    ss << hspace << "    to:" << std::endl;
    for(const Vertex::Ptr& to : getTargetVertices())
    {
        ss << to->toString(indent + 8) << std::endl;
    }
    return ss.str();
}




} // end namespace constraints
} // end namespace templ
