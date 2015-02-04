#ifndef TEREP_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP
#define TEREP_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP

#include <terep/solvers/Constraint.hpp>
#include <terep/solvers/temporal/point_algebra/TimePoint.hpp>

namespace terep {
namespace solvers {
namespace temporal {
namespace point_algebra {

enum QualitativeConstraintType { Empty, Greater, Less, Equal, GreaterOrEqual, LessOrEqual, Universal };
extern std::map<QualitativeConstraintType, QualitativeConstraintType> QualitativeConstraintSymmetricType;

extern std::map<QualitativeConstraintType, std::string> QualitativeConstraintTypeTxt;

class QualitativeTimePointConstraint : public Constraint
{
    QualitativeConstraintType mConstraintType;
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
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace terep

#endif // TEREP_SOLVERS_TEMPORAL_TIMEPOINT_CONSTRAINT_HPP
