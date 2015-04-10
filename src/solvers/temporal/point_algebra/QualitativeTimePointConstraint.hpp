#ifndef TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP

#include <templ/solvers/Constraint.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

/// Primitive relations are P = {<,=,>} and timepoints can be related in only
/// these three ways
/// Constraining can be done in additional ways, i.e. <=, >=, !=
///
/// Here we rely on the qualititive constraints
/// R =2^P = { {}{<},{=},{>},{<,=},{>,=},{<,>},P}
///
enum QualitativeConstraintType { Empty, Greater, Less, Equal, Distinct, GreaterOrEqual, LessOrEqual, Universal, QualitativeConstraintTypeEndMarker };

/// Symmetric constraint: change the primitive '<' by '>' and vice versa
extern std::map<QualitativeConstraintType, QualitativeConstraintType> QualitativeConstraintSymmetricType;
extern std::map<QualitativeConstraintType, std::string> QualitativeConstraintTypeTxt;

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
extern std::map< std::pair<QualitativeConstraintType,QualitativeConstraintType>, QualitativeConstraintType > QualitativeConstraintTypeAlgebra;

/**
 * A qualitative timepoint constraint does not take into account bounds of any
 * timepoint. 
 * Thus one should use instances of QualitativeTimePoint when working with this
 * kind of constraint
 */
class QualitativeTimePointConstraint : public Constraint
{
    QualitativeConstraintType mConstraintType;

    static bool hasIntersection(QualitativeConstraintType firstType, QualitativeConstraintType secondType);

public:
    typedef boost::shared_ptr<QualitativeTimePointConstraint> Ptr;

    QualitativeTimePointConstraint(Variable::Ptr source, Variable::Ptr target, point_algebra::QualitativeConstraintType constraintType);
    
    virtual std::string getClassName() const { return "QualitativeTimePointConstraint"; }

    QualitativeConstraintType getQualitativeConstraintType() const { return mConstraintType; }

    static QualitativeTimePointConstraint::Ptr create(Variable::Ptr source, Variable::Ptr target,QualitativeConstraintType constraintType);

    static QualitativeTimePointConstraint::Ptr getSymmetric(QualitativeTimePointConstraint::Ptr constraint);

    /**
     * Get the symmetric constraint
     */
    static QualitativeConstraintType getSymmetric(QualitativeConstraintType type);

    static QualitativeConstraintType getComposition(QualitativeConstraintType firstType, QualitativeConstraintType secondType);

    /**
     * Check if a constraint type is consistent with another
     */
    static bool isConsistent(QualitativeConstraintType firstType, QualitativeConstraintType secondType);

    /**
     * Retrieve a list that contains all available constraint types
     * \return constraint type list
     */
    static std::vector<QualitativeConstraintType> getAllConstraintTypes();
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ

#endif // TEMPL_SOLVERS_TEMPORAL_TIMEPOINT_CONSTRAINT_HPP
