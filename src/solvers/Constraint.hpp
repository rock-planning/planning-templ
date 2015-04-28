#ifndef TEMPL_SOLVERS_CONSTRAINT_HPP
#define TEMPL_SOLVERS_CONSTRAINT_HPP

#include <templ/solvers/Variable.hpp>
#include <graph_analysis/Edge.hpp>

#define T_CONSTRAINT(x) boost::dynamic_pointer_cast< templ::solvers::Constraint>(x)

namespace templ {
namespace solvers {

class Constraint : public graph_analysis::Edge
{
public:
    typedef boost::shared_ptr<Constraint> Ptr;

    Constraint(Variable::Ptr source, Variable::Ptr target);

    virtual ~Constraint() {}

    virtual std::string getClassName() const { return "Constraint"; }

    virtual std::string toString() const;

    Variable::Ptr getSourceVariable() { return boost::dynamic_pointer_cast<Variable>( getSourceVertex()); }
    Variable::Ptr getTargetVariable() { return boost::dynamic_pointer_cast<Variable>( getTargetVertex()); }

protected:
    virtual graph_analysis::Edge* doClone() { return new Constraint(*this); }
};

typedef std::vector<Constraint::Ptr> ConstraintList;

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CONSTRAINT_HPP
