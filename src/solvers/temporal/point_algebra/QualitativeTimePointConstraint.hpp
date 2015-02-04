#ifndef TEREP_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP
#define TEREP_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP

#include <terep/solvers/Constraint.hpp>
#include <terep/solvers/temporal/point_algebra/TimePoint.hpp>

namespace terep {
namespace solvers {
namespace temporal {
namespace point_algebra {

/// Primitive relations are P = {<,=,>} and timepoint can be related in only
/// these three ways
/// Constraining can be done in additional way,s: <=, >=, !=
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
///  o| < | = | >
///  <| < | < | P
///  =| < | = | >
///  >| P | > | >
///
/// The operator is associative and distributive, i.e.
/// {<,=} o {>,=} = {<}o{>} \cup {<}o{=} \cup {=}o{>} \cup {=}o{>} = {P} \cup
/// {>} \cup {<} \cup {>} = {P}
extern std::map< std::pair<QualitativeConstraintType,QualitativeConstraintType>, QualitativeConstraintType > QualitativeConstraintTypeAlgebra;

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

    static bool isConsistent(QualitativeConstraintType firstType, QualitativeConstraintType secondType);

    static std::vector<QualitativeConstraintType> getAllConstraintTypes();
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace terep

#endif // TEREP_SOLVERS_TEMPORAL_TIMEPOINT_CONSTRAINT_HPP
