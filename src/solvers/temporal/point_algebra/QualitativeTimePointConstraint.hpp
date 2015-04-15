#ifndef TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP

#include <templ/solvers/Constraint.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

/**
 * A qualitative timepoint constraint does not take into account bounds of any
 * timepoint. 
 * Thus one should use instances of QualitativeTimePoint when working with this
 * kind of constraint
 */
class QualitativeTimePointConstraint : public Constraint
{
public:
    /// Primitive relations are P = {<,=,>} and timepoints can be related in only
    /// these three ways
    /// Constraining can be done in additional ways, i.e. <=, >=, !=
    ///
    /// Here we rely on the qualititive constraints
    /// R =2^P = { {}{<},{=},{>},{<,=},{>,=},{<,>},P}
    ///
    enum Type { Empty, Greater, Less, Equal, Distinct, GreaterOrEqual, LessOrEqual, Universal, TypeEndMarker };

    typedef boost::shared_ptr<QualitativeTimePointConstraint> Ptr;

    QualitativeTimePointConstraint(Variable::Ptr source, Variable::Ptr target, Type constraintType);
    
    virtual std::string getClassName() const { return "QualitativeTimePointConstraint"; }

    Type getType() const { return mConstraintType; }

    static QualitativeTimePointConstraint::Ptr create(Variable::Ptr source, Variable::Ptr target, Type constraintType);

    static QualitativeTimePointConstraint::Ptr getSymmetric(QualitativeTimePointConstraint::Ptr constraint);

    /**
     * Get the symmetric constraint
     */
    static Type getSymmetric(Type type);

    static Type getComposition(Type firstType, Type secondType);

    /**
     * Check if a constraint type is consistent with another
     */
    static bool isConsistent(Type firstType, Type secondType);

    /**
     * Retrieve a list that contains all available constraint types
     * \return constraint type list
     */
    static std::vector<Type> getAllConstraintTypes();

    /// Symmetric constraint: change the primitive '<' by '>' and vice versa
    static std::map<Type, Type> SymmetricType;
    static std::map<Type, std::string> TypeTxt;

    /// Composition table for the time-point algebra for '<','=','>' and the
    /// additional '<=','>='
    ///
    /// \see "Automated Planning - Theory and Practice" (Chap. 13.3 Qualitative
    ///       Temporal Relations)
    ///
    ///  o| < | = | >
    ///  <| < | < | P
    ///  =| < | = | >
    ///  >| P | > | >
    ///
    /// The operator o is associative and distributive, i.e.
    /// {<,=} o {>,=} = {<}o{>} \cup {<}o{=} \cup {=}o{>} \cup {=}o{>} = {P} \cup
    /// {>} \cup {<} \cup {>} = {P}
    static std::map< std::pair<Type,Type>, Type > TypeAlgebra;

private:
    Type mConstraintType;

    static bool hasIntersection(Type firstType, Type secondType);
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_TIMEPOINT_CONSTRAINT_HPP
