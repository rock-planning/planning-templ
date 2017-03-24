#ifndef TEMPL_SOLVERS_VARIABLE_HPP
#define TEMPL_SOLVERS_VARIABLE_HPP

#include <graph_analysis/Vertex.hpp>
#include <templ/SharedPtr.hpp>

#define T_VAR(x) dynamic_pointer_cast< templ::solvers::Variable>(x)

namespace templ {
namespace solvers {

/**
 * The general representation of a constraint variable in the constraint network
 */
class Variable : public graph_analysis::Vertex
{
public:
    virtual ~Variable() {}

    typedef shared_ptr<Variable> Ptr;

    /**
     * Get classname
     */
    virtual std::string getClassName() const { return "Variable"; }

    /**
     * Get stringified object
     */
    virtual std::string toString() const;

protected:
    virtual graph_analysis::Vertex* doClone() { return new Variable(*this); }
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_VARIABLE_HPP
