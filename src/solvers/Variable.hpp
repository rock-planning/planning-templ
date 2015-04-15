#ifndef TEMPL_SOLVERS_VARIABLE_HPP
#define TEMPL_SOLVERS_VARIABLE_HPP

#include <graph_analysis/Vertex.hpp>

#define T_VAR(x) boost::dynamic_pointer_cast< templ::solvers::Variable>(x)

namespace templ {
namespace solvers {

class Variable : public graph_analysis::Vertex
{
public:
    virtual ~Variable() {}

    typedef boost::shared_ptr<Variable> Ptr;

    virtual std::string getClassName() const { return "Variable"; }

    virtual std::string toString() const;
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_VARIABLE_HPP
