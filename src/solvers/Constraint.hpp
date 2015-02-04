#ifndef TEREP_SOLVERS_CONSTRAINT_HPP
#define TEREP_SOLVERS_CONSTRAINT_HPP

#include <terep/solvers/Variable.hpp>
#include <graph_analysis/Edge.hpp>

namespace terep {
namespace solvers {

class Constraint : public graph_analysis::Edge
{
public:
    typedef boost::shared_ptr<Constraint> Ptr;

    Constraint(Variable::Ptr source, Variable::Ptr target);

    virtual std::string getClassName() const { return "Constraint"; }

    virtual std::string toString() const;

    Variable::Ptr getSourceVariable() { return boost::dynamic_pointer_cast<Variable>( getSourceVertex()); }
    Variable::Ptr getTargetVariable() { return boost::dynamic_pointer_cast<Variable>( getTargetVertex()); }
};

} // end namespace solvers
} // end namespace terep
#endif // TEREP_SOLVERS_CONSTRAINT_HPP
