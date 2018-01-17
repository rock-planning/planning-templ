#ifndef TEMPL_HYPER_CONSTRAINT_HPP
#define TEMPL_HYPER_CONSTRAINT_HPP

#include <graph_analysis/DirectedHyperEdge.hpp>
#include "../Constraint.hpp"

namespace templ {
namespace constraints {

/**
 * A hyper constraint allows to cover multiple vertices with a single constraint
 * This is for example the case when a constraint covers an interval in the SpaceTime Network
 */
class HyperConstraint : public Constraint, public graph_analysis::DirectedHyperEdge
{
public:
    typedef shared_ptr<Constraint> Ptr;

    /**
     * Initialize the HyperConstraint and mark with one of the
     * available constraint categories
     * \param category A constraint category such as MODEL or
     * TEMPORAL_QUANTITATIVE
     */
    HyperConstraint(Category category = UNKNOWN);

    /**
     * Initialize the HyperConstraint, if the constraint is not directed,
     * then source should contain all relevant Variables
     * \param category A constraint category such as MODEL or
     * TEMPORAL_QUANTITATIVE
     * \param sources If the constraint is directed defines the sources
     * \param targets if the constraint is directed defines the targets
     */
    HyperConstraint(Category category, const Variable::PtrList& sources, const Variable::PtrList& targets);

    virtual ~HyperConstraint();

    virtual std::string getClassName() const override { return "HyperConstraint"; }

    virtual std::string toString() const override;

    virtual std::string toString(uint32_t indent) const override;

    /**
     * Get list of source variables that a associated with this hyper constraint
     */
    Variable::PtrList getSourceVariables() const;

    /**
     * Get list of target variables that are associated with this hyper
     * constraint
     */
    Variable::PtrList getTargetVariables() const;

protected:

    /// Make sure the constraint is cloneable
    graph_analysis::Vertex* getClone() const override { return new HyperConstraint(*this); }
};

} // end namespace constraints
} // end namespace templ
#endif // TEMPL_HYPER_CONSTRAINT_HPP
