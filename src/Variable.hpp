#ifndef TEMPL_VARIABLE_HPP
#define TEMPL_VARIABLE_HPP

#include <graph_analysis/Vertex.hpp>
#include "SharedPtr.hpp"

#define T_VAR(x) dynamic_pointer_cast< templ::Variable>(x)

namespace templ {

/**
 * The general representation of a constraint variable in the constraint network
 */
class Variable : public graph_analysis::Vertex
{
public:
    Variable(const std::string& label = std::string());
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
    virtual graph_analysis::Vertex* getClone() const { return new Variable(*this); }
};

} // end namespace templ
#endif // TEMPL_VARIABLE_HPP
