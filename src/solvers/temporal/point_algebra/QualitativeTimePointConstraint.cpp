#include "QualitativeTimePointConstraint.hpp"
#include <boost/assign/list_of.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

std::map<QualitativeTimePointConstraint::Type, std::string> QualitativeTimePointConstraint::TypeTxt = boost::assign::map_list_of
    (QualitativeTimePointConstraint::Empty, "{}")
    (QualitativeTimePointConstraint::Greater, ">")
    (QualitativeTimePointConstraint::Less, "<")
    (QualitativeTimePointConstraint::Equal, "=")
    (QualitativeTimePointConstraint::Distinct, "!=")
    (QualitativeTimePointConstraint::GreaterOrEqual, ">=")
    (QualitativeTimePointConstraint::LessOrEqual, "<=")
    (QualitativeTimePointConstraint::Universal, "P")
    ;

std::map<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type> QualitativeTimePointConstraint::SymmetricType = boost::assign::map_list_of
    (QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::Empty)
    (QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::Less)
    (QualitativeTimePointConstraint::Less, QualitativeTimePointConstraint::Greater)
    (QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::Equal)
    (QualitativeTimePointConstraint::Distinct, QualitativeTimePointConstraint::Distinct)
    (QualitativeTimePointConstraint::GreaterOrEqual, QualitativeTimePointConstraint::LessOrEqual)
    (QualitativeTimePointConstraint::LessOrEqual, QualitativeTimePointConstraint::GreaterOrEqual)
    (QualitativeTimePointConstraint::Universal, QualitativeTimePointConstraint::Universal)
    ;

std::map< std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>, QualitativeTimePointConstraint::Type> QualitativeTimePointConstraint::TypeAlgebra = boost::assign::map_list_of
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Less, QualitativeTimePointConstraint::Less), QualitativeTimePointConstraint::Less)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Less, QualitativeTimePointConstraint::Equal), QualitativeTimePointConstraint::Less)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Less, QualitativeTimePointConstraint::Greater), QualitativeTimePointConstraint::Universal)
    // {<}o{<,=} --> {<}o{<} \cup {<}o{=} = {<} \cup {<}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Less, QualitativeTimePointConstraint::LessOrEqual), QualitativeTimePointConstraint::Less)
    // {<}o{>,=} --> {<}o{>} \cup {<}o{=} = {P} \cup {<}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Less, QualitativeTimePointConstraint::GreaterOrEqual), QualitativeTimePointConstraint::Universal)

    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::Less), QualitativeTimePointConstraint::Less)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::Equal), QualitativeTimePointConstraint::Equal)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::Greater), QualitativeTimePointConstraint::Greater)
    // {=}o{<,=} --> {=}o{<} \cup {=}o{=} = {<} \cup {=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::LessOrEqual), QualitativeTimePointConstraint::LessOrEqual)
    // {=}o{>,=} --> {=}o{>} \cup {=}o{=} = {>} \cup {=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::GreaterOrEqual), QualitativeTimePointConstraint::GreaterOrEqual)

    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::Less), QualitativeTimePointConstraint::Universal)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::Equal), QualitativeTimePointConstraint::Greater)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::Greater), QualitativeTimePointConstraint::Greater)
    // {>}o{<,=} --> {>}o{<} \cup {>}o{=} = {P} \cup {>}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::LessOrEqual), QualitativeTimePointConstraint::Universal)
    // {>}o{>,=} --> {>}o{>} \cup {>}o{=} = {>} \cup {>}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::GreaterOrEqual), QualitativeTimePointConstraint::Greater )

    // {>,=}o{<} --> {>}o{<} \cup {=}o{<} = {P} \cup {<}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::GreaterOrEqual, QualitativeTimePointConstraint::Less), QualitativeTimePointConstraint::Universal )
    // {>,=}o{=} --> {>}o{=} \cup {=}o{=} = {>} \cup {=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::GreaterOrEqual, QualitativeTimePointConstraint::Equal), QualitativeTimePointConstraint::GreaterOrEqual )
    // {>,=}o{>} --> {>}o{=} \cup {=}o{>} = {>} \cup {>}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::GreaterOrEqual, QualitativeTimePointConstraint::Greater), QualitativeTimePointConstraint::Greater )
    // {>,=}o{<,=} --> {>}o{<} \cup {>}o{=} \cup {=}o{<} \cup {=}o{=} = {P} \cup {>} \cup {<} \cup {=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::GreaterOrEqual, QualitativeTimePointConstraint::LessOrEqual), QualitativeTimePointConstraint::Universal)
    // {>,=}o{>,=} --> {>}o{>} \cup {>}o{=} \cup {=}o{>} \cup {=}o{=} = {>} \cup {>} \cup {>} \cup {=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::GreaterOrEqual, QualitativeTimePointConstraint::GreaterOrEqual), QualitativeTimePointConstraint::GreaterOrEqual )

    // {<,=}o{<} --> {<}o{<} \cup {=}o{<} = {<} \cup {<}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::LessOrEqual, QualitativeTimePointConstraint::Less), QualitativeTimePointConstraint::Less )
    // {<,=}o{=} --> {<}o{=} \cup {=}o{=} = {<} \cup {=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::LessOrEqual, QualitativeTimePointConstraint::Equal), QualitativeTimePointConstraint::LessOrEqual)
    // {<,=}o{>} --> {<}o{>} \cup {=}o{>} = {P} \cup {>}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::LessOrEqual, QualitativeTimePointConstraint::Greater), QualitativeTimePointConstraint::Universal)
    // {<,=}o{<,=} --> {<}o{<} \cup {=}o{=} \cup {=}o{<} \cup {=}o{=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::LessOrEqual, QualitativeTimePointConstraint::LessOrEqual), QualitativeTimePointConstraint::LessOrEqual)
    // {<,=}o{>,=} --> {<}o{>} \cup {<}o{=} \cup {=}o{>} \cup {=}o{=} = {P} \cup
    // {<} \cup {>} \cup{=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::LessOrEqual, QualitativeTimePointConstraint::GreaterOrEqual), QualitativeTimePointConstraint::Universal )
    ;

QualitativeTimePointConstraint::QualitativeTimePointConstraint(Variable::Ptr source, Variable::Ptr target, point_algebra::QualitativeTimePointConstraint::Type constraintType)
    : Constraint(source, target)
    , mConstraintType(constraintType)
{}


QualitativeTimePointConstraint::Ptr QualitativeTimePointConstraint::create(Variable::Ptr source, Variable::Ptr target, point_algebra::QualitativeTimePointConstraint::Type constraintType)
{
    return QualitativeTimePointConstraint::Ptr( new QualitativeTimePointConstraint(source, target, constraintType) );
}

QualitativeTimePointConstraint::Ptr QualitativeTimePointConstraint::getSymmetric(QualitativeTimePointConstraint::Ptr constraint)
{
    return QualitativeTimePointConstraint::Ptr( new QualitativeTimePointConstraint(constraint->getTargetVariable(), constraint->getSourceVariable(), getSymmetric(constraint->getType())) );
}

QualitativeTimePointConstraint::Type QualitativeTimePointConstraint::getSymmetric(QualitativeTimePointConstraint::Type type)
{
    std::map<Type, Type>::const_iterator cit = SymmetricType.find(type); 
    if(cit == SymmetricType.end())
    {
        throw std::runtime_error("QualitativeTimePointConstraint::getSymmetric: no symmetric type defined");
    }
    return cit->second;
}

QualitativeTimePointConstraint::Type QualitativeTimePointConstraint::getComposition(QualitativeTimePointConstraint::Type firstType, QualitativeTimePointConstraint::Type secondType)
{
    if(firstType == Universal || secondType == Universal)
        return Universal;

    std::pair<Type, Type> key(firstType, secondType);
    std::map< std::pair<Type, Type>, Type >::const_iterator cit = TypeAlgebra.find(key);
    if(cit == TypeAlgebra.end())
    {
        throw std::runtime_error("QualitativeTimePointConstraint::getComposition found no match for composition of: " + TypeTxt[firstType] + " and " + TypeTxt[secondType]);
    }

    return cit->second;
}

QualitativeTimePointConstraint::Type QualitativeTimePointConstraint::getComposition(const std::vector<QualitativeTimePointConstraint::Type>& typeList)
{
    std::vector<Type>::const_iterator cit = typeList.begin();
    Type compositionType = Empty;
    for(; cit != typeList.end(); ++cit)
    {
        compositionType = getComposition(compositionType, *cit);
    }
    return compositionType;
}

bool QualitativeTimePointConstraint::isConsistent(QualitativeTimePointConstraint::Type firstType, QualitativeTimePointConstraint::Type secondType)
{
    return hasIntersection(firstType, secondType);
}

bool QualitativeTimePointConstraint::hasIntersection(QualitativeTimePointConstraint::Type firstType, QualitativeTimePointConstraint::Type secondType)
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

std::vector<QualitativeTimePointConstraint::Type> QualitativeTimePointConstraint::getAllConstraintTypes()
{
    std::vector<QualitativeTimePointConstraint::Type> types;
    for(int i = (int) Empty; i < QualitativeTimePointConstraint::TypeEndMarker; ++i)
    {
        types.push_back((QualitativeTimePointConstraint::Type) i);
    }
    return types;
}

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
