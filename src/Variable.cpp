#include "Variable.hpp"

namespace templ {

const graph_analysis::VertexRegistration<Variable> Variable::msRegistration;

Variable::Variable(const std::string& label)
    : graph_analysis::Vertex(label)
{}

std::string Variable::toString() const
{
    return getClassName();
}

} // end namespace templ
