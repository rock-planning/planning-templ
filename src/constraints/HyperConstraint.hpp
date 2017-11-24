#ifndef TEMPL_HYPER_CONSTRAINT_HPP
#define TEMPL_HYPER_CONSTRAINT_HPP

#include <graph_analysis/DirectedHyperEdge.hpp>
#include "../Constraint.hpp"

namespace templ {
namespace constraints {

class HyperConstraint : public Constraint, public graph_analysis::DirectedHyperEdge
{
public:
    typedef shared_ptr<Constraint> Ptr;

    HyperConstraint(Type type = UNKNOWN);

    HyperConstraint(Type type, const Variable::PtrList& sources, const Variable::PtrList& targets);

    virtual ~HyperConstraint();

    virtual std::string getClassName() const override { return "HyperConstraint"; }

    virtual std::string toString() const override;

    virtual std::string toString(uint32_t indent) const override;

    Variable::PtrList getSourceVariables() const;

    Variable::PtrList getTargetVariables() const;

protected:

    /// Make
    graph_analysis::Vertex* getClone() const override { return new HyperConstraint(*this); }
};

} // end namespace constraints
} // end namespace templ
#endif // TEMPL_HYPER_CONSTRAINT_HPP
