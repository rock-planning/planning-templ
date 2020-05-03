#include "Variable.hpp"

namespace templ {

Variable::Variable(const std::string& label)
    : graph_analysis::Vertex(label)
{}

std::string Variable::toString() const
{
    return getClassName();
}

} // end namespace templ
