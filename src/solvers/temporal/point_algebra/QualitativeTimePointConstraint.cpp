#include "QualitativeTimePointConstraint.hpp"
#include <boost/assign/list_of.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

std::map<QualitativeConstraintType, std::string> QualitativeConstraintTypeTxt = boost::assign::map_list_of
    (Empty, "{}")
    (Greater, ">")
    (Less, "<")
    (Equal, "=")
    (Distinct, "!=")
    (GreaterOrEqual, ">=")
    (LessOrEqual, "<=")
    (Universal, "P")
    ;

std::map<QualitativeConstraintType, QualitativeConstraintType> QualitativeConstraintSymmetricType = boost::assign::map_list_of
    (Empty, Empty)
    (Greater, Less)
    (Less, Greater)
    (Equal, Equal)
    (Distinct, Distinct)
    (GreaterOrEqual, LessOrEqual)
    (LessOrEqual, GreaterOrEqual)
    (Universal, Universal)
    ;

std::map< std::pair<QualitativeConstraintType, QualitativeConstraintType>, QualitativeConstraintType> QualitativeConstraintTypeAlgebra = boost::assign::map_list_of
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Less, Less), Less)
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Less, Equal), Less)
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Less, Greater), Universal)
    // {<}o{<,=} --> {<}o{<} \cup {<}o{=} = {<} \cup {<}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Less, LessOrEqual), Less)
    // {<}o{>,=} --> {<}o{>} \cup {<}o{=} = {P} \cup {<}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Less, GreaterOrEqual), Universal)

    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Equal, Less), Less)
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Equal, Equal), Equal)
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Equal, Greater), Greater)
    // {=}o{<,=} --> {=}o{<} \cup {=}o{=} = {<} \cup {=}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Equal, LessOrEqual), LessOrEqual)
    // {=}o{>,=} --> {=}o{>} \cup {=}o{=} = {>} \cup {=}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Equal, GreaterOrEqual), GreaterOrEqual)

    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Greater, Less), Universal)
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Greater, Equal), Greater)
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Greater, Greater), Greater)
    // {>}o{<,=} --> {>}o{<} \cup {>}o{=} = {P} \cup {>}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Greater, LessOrEqual), Universal)
    // {>}o{>,=} --> {>}o{>} \cup {>}o{=} = {>} \cup {>}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(Greater, GreaterOrEqual), Greater )

    // {>,=}o{<} --> {>}o{<} \cup {=}o{<} = {P} \cup {<}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(GreaterOrEqual, Less), Universal )
    // {>,=}o{=} --> {>}o{=} \cup {=}o{=} = {>} \cup {=}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(GreaterOrEqual, Equal), GreaterOrEqual )
    // {>,=}o{>} --> {>}o{=} \cup {=}o{>} = {>} \cup {>}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(GreaterOrEqual, Greater), Greater )
    // {>,=}o{<,=} --> {>}o{<} \cup {>}o{=} \cup {=}o{<} \cup {=}o{=} = {P} \cup {>} \cup {<} \cup {=}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(GreaterOrEqual, LessOrEqual), Universal)
    // {>,=}o{>,=} --> {>}o{>} \cup {>}o{=} \cup {=}o{>} \cup {=}o{=} = {>} \cup {>} \cup {>} \cup {=}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(GreaterOrEqual, GreaterOrEqual), GreaterOrEqual )

    // {<,=}o{<} --> {<}o{<} \cup {=}o{<} = {<} \cup {<}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(LessOrEqual, Less), Less )
    // {<,=}o{=} --> {<}o{=} \cup {=}o{=} = {<} \cup {=}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(LessOrEqual, Equal), LessOrEqual)
    // {<,=}o{>} --> {<}o{>} \cup {=}o{>} = {P} \cup {>}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(LessOrEqual, Greater), Universal)
    // {<,=}o{<,=} --> {<}o{<} \cup {=}o{=} \cup {=}o{<} \cup {=}o{=}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(LessOrEqual, LessOrEqual), LessOrEqual)
    // {<,=}o{>,=} --> {<}o{>} \cup {<}o{=} \cup {=}o{>} \cup {=}o{=} = {P} \cup
    // {<} \cup {>} \cup{=}
    ( std::pair<QualitativeConstraintType, QualitativeConstraintType>(LessOrEqual, GreaterOrEqual), Universal )
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

QualitativeConstraintType QualitativeTimePointConstraint::getComposition(QualitativeConstraintType firstType, QualitativeConstraintType secondType)
{
    if(firstType == Universal || secondType == Universal)
        return Universal;

    std::pair<QualitativeConstraintType, QualitativeConstraintType> key(firstType, secondType);
    std::map< std::pair<QualitativeConstraintType,QualitativeConstraintType>, QualitativeConstraintType >::const_iterator cit = QualitativeConstraintTypeAlgebra.find(key);
    if(cit == QualitativeConstraintTypeAlgebra.end())
    {
        throw std::runtime_error("QualitativeTimePointConstraint::getComposition found no match for composition of: " + QualitativeConstraintTypeTxt[firstType] + " and " + QualitativeConstraintTypeTxt[secondType]);
    }

    return cit->second;
}

bool QualitativeTimePointConstraint::isConsistent(QualitativeConstraintType firstType, QualitativeConstraintType secondType)
{
    return hasIntersection(firstType, secondType);
}

bool QualitativeTimePointConstraint::hasIntersection(QualitativeConstraintType firstType, QualitativeConstraintType secondType)
{
    // This implementation is actually the verbose way of intersecting the set of primitive
    // operators
    
    if(firstType != Empty && secondType == Universal)
    {
        return true;
    } else if (firstType == Universal && secondType != Empty)
    {
        return true;
    } else if(firstType == Empty || secondType == Empty)
    {
        return false;
    }

    switch(firstType)
    {
        case Less:
            {
                switch(secondType)
                {
                    case Less:
                    case LessOrEqual:
                        return true;
                    default:
                        return false;
                }
            }
        case Greater:
            {
                switch(secondType)
                {
                    case Greater:
                    case GreaterOrEqual:
                        return true;
                    default:
                        return false;
                }
            }
        case Equal:
            {
                switch(secondType)
                {
                    case GreaterOrEqual:
                    case LessOrEqual:
                    case Equal:
                        return true;
                    default:
                        return false;
                }
            }
        case LessOrEqual:
            {
                switch(secondType)
                {
                    case Less:
                    case LessOrEqual:
                    case GreaterOrEqual:
                    case Equal:
                        return true;
                    default:
                        return false;
                }
            }
        case GreaterOrEqual:
            {
                switch(secondType)
                {
                    case Greater:
                    case GreaterOrEqual:
                    case LessOrEqual:
                    case Equal:
                        return true;
                    default:
                        return false;
                }
            }
        default:
            return false;
    }
}

std::vector<QualitativeConstraintType> QualitativeTimePointConstraint::getAllConstraintTypes()
{
    std::vector<QualitativeConstraintType> types;
    for(int i = (int) Empty; i < QualitativeConstraintTypeEndMarker; ++i)
    {
        types.push_back((QualitativeConstraintType) i);
    }
    return types;
}

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
