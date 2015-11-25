#ifndef TEMPL_SOLVERS_CONSTRAINT_HPP
#define TEMPL_SOLVERS_CONSTRAINT_HPP

#include <templ/solvers/Variable.hpp>
#include <graph_analysis/Edge.hpp>

#define T_CONSTRAINT(x) dynamic_pointer_cast<templ::solvers::Constraint>(x)

namespace templ {
namespace solvers {

/**
 * A Constraint represents an edge in the constraint network
 */
class Constraint : public graph_analysis::Edge
{
public:
    typedef shared_ptr<Constraint> Ptr;

    /**
     * Default constructor for a constraint
     */
    Constraint(Variable::Ptr source, Variable::Ptr target);

    virtual ~Constraint() {}

    /**
     * Get the class name of this constraint
     * \return classname
     */
    virtual std::string getClassName() const { return "Constraint"; }

    /**
     * Get stringified object
     * \return string repr
     */
    virtual std::string toString() const;

    /**
     * Get the source vertex/variable of this constraint
     */
    Variable::Ptr getSourceVariable() { return dynamic_pointer_cast<Variable>( getSourceVertex()); }

    /**
     * Get the target vertex/variable of this constraint
     */
    Variable::Ptr getTargetVariable() { return dynamic_pointer_cast<Variable>( getTargetVertex()); }

protected:
    /// Make sure cloning works for this constraint
    virtual graph_analysis::Edge* doClone() { return new Constraint(*this); }
};

typedef std::vector<Constraint::Ptr> ConstraintList;

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CONSTRAINT_HPP
