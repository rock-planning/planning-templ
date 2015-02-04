#include "QualitativeTimePointConstraint.hpp"
#include <boost/assign/list_of.hpp>

namespace terep {
namespace solvers {
namespace temporal {
namespace point_algebra {

std::map<QualitativeConstraintType, std::string> QualitativeConstraintTypeTxt = boost::assign::map_list_of
    (Empty, "{}")
    (Greater, ">")
    (Less, "<")
    (Equal, "=")
    (GreaterOrEqual, ">=")
    (LessOrEqual, "<=")
    (Universal, "P")
    ;

std::map<QualitativeConstraintType, QualitativeConstraintType> QualitativeConstraintSymmetricType = boost::assign::map_list_of
    (Empty, Empty)
    (Greater, LessOrEqual)
    (Less, GreaterOrEqual)
    (Equal, Equal)
    (GreaterOrEqual, Less)
    (LessOrEqual, Greater)
    (Universal, Universal)
    ;

QualitativeTimePointConstraint::QualitativeTimePointConstraint(Variable::Ptr source, Variable::Ptr target, point_algebra::QualitativeConstraintType constraintType)
    : Constraint(source, target)
    , mConstraintType(constraintType)
{}


QualitativeTimePointConstraint::Ptr QualitativeTimePointConstraint::create(Variable::Ptr source, Variable::Ptr target, point_algebra::QualitativeConstraintType constraintType)
{
    return QualitativeTimePointConstraint::Ptr( new QualitativeTimePointConstraint(source, target, constraintType) );
}

QualitativeTimePointConstraint::Ptr QualitativeTimePointConstraint::getSymmetric(QualitativeTimePointConstraint::Ptr constraint)
{
    return QualitativeTimePointConstraint::Ptr( new QualitativeTimePointConstraint(constraint->getTargetVariable(), constraint->getSourceVariable(), getSymmetric(constraint->getQualitativeConstraintType())) );
}

QualitativeConstraintType QualitativeTimePointConstraint::getSymmetric(QualitativeConstraintType type)
{
    std::map<QualitativeConstraintType, QualitativeConstraintType>::const_iterator cit = QualitativeConstraintSymmetricType.find(type); 
    if(cit == QualitativeConstraintSymmetricType.end())
    {
        throw std::runtime_error("QualitativeTimePointConstraint::getSymmetric: no symmetric type defined");
    }
    return cit->second;
}


} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace terep
