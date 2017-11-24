#include "QualitativeTimePointConstraint.hpp"
#include <boost/assign/list_of.hpp>
#include <base-logging/Logging.hpp>

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

bool QualitativeTimePointConstraint::msInitializedCompositionTable = false;
bool QualitativeTimePointConstraint::msInitializedSymmetryTable = false;
QualitativeTimePointConstraint::Type QualitativeTimePointConstraint::msCompositionTable[8][8];
QualitativeTimePointConstraint::Type QualitativeTimePointConstraint::msSymmetryTable[8];

std::map<QualitativeTimePointConstraint::Type, std::string> QualitativeTimePointConstraint::TypeTxt = boost::assign::map_list_of
    (QualitativeTimePointConstraint::Empty, "{}")
    (QualitativeTimePointConstraint::Greater, ">")
    (QualitativeTimePointConstraint::Less, "<")
    (QualitativeTimePointConstraint::Equal, "=")
    //(QualitativeTimePointConstraint::Distinct, "!=") // > <
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
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Less, QualitativeTimePointConstraint::Distinct), QualitativeTimePointConstraint::Distinct)

    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::Less), QualitativeTimePointConstraint::Less)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::Equal), QualitativeTimePointConstraint::Equal)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::Greater), QualitativeTimePointConstraint::Greater)
    // {=}o{<,=} --> {=}o{<} \cup {=}o{=} = {<} \cup {=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::LessOrEqual), QualitativeTimePointConstraint::LessOrEqual)
    // {=}o{>,=} --> {=}o{>} \cup {=}o{=} = {>} \cup {=}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::GreaterOrEqual), QualitativeTimePointConstraint::GreaterOrEqual)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Equal, QualitativeTimePointConstraint::Distinct), QualitativeTimePointConstraint::Universal )

    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::Less), QualitativeTimePointConstraint::Universal)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::Equal), QualitativeTimePointConstraint::Greater)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::Greater), QualitativeTimePointConstraint::Greater)
    // {>}o{<,=} --> {>}o{<} \cup {>}o{=} = {P} \cup {>}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::LessOrEqual), QualitativeTimePointConstraint::Universal)
    // {>}o{>,=} --> {>}o{>} \cup {>}o{=} = {>} \cup {>}
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::GreaterOrEqual), QualitativeTimePointConstraint::Greater )
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Greater, QualitativeTimePointConstraint::Distinct), QualitativeTimePointConstraint::Distinct )

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
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::GreaterOrEqual, QualitativeTimePointConstraint::Distinct), QualitativeTimePointConstraint::Universal )

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
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::LessOrEqual, QualitativeTimePointConstraint::Distinct), QualitativeTimePointConstraint::Universal )

    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::Empty), QualitativeTimePointConstraint::Empty)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::Less), QualitativeTimePointConstraint::Less)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::LessOrEqual), QualitativeTimePointConstraint::LessOrEqual)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::Greater), QualitativeTimePointConstraint::Greater )
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::GreaterOrEqual), QualitativeTimePointConstraint::GreaterOrEqual)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::Equal), QualitativeTimePointConstraint::Equal)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::Distinct), QualitativeTimePointConstraint::Distinct)
    ( std::pair<QualitativeTimePointConstraint::Type, QualitativeTimePointConstraint::Type>(QualitativeTimePointConstraint::Empty, QualitativeTimePointConstraint::Universal), QualitativeTimePointConstraint::Universal)
    ;

QualitativeTimePointConstraint::QualitativeTimePointConstraint()
    : SimpleConstraint(Constraint::TEMPORAL_QUALITATIVE)
    , mConstraintType(Empty)
{
    SimpleConstraint::setLabel(TypeTxt[mConstraintType]);
}

QualitativeTimePointConstraint::QualitativeTimePointConstraint(Variable::Ptr source, Variable::Ptr target, point_algebra::QualitativeTimePointConstraint::Type constraintType)
    : SimpleConstraint(Constraint::TEMPORAL_QUALITATIVE, source, target)
    , mConstraintType(constraintType)
{
    SimpleConstraint::setLabel(TypeTxt[constraintType]);
}

void QualitativeTimePointConstraint::setLabel(const std::string& label)
{
    std::map<QualitativeTimePointConstraint::Type, std::string>::const_iterator cit = TypeTxt.begin();
    for(; cit != TypeTxt.end(); ++cit)
    {
        if(cit->second == label)
        {
            mConstraintType = cit->first;
            SimpleConstraint::setLabel(label);
            return;
        }
    }
    throw std::invalid_argument("templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::setLabel: Labels are limited to valid constraint values, e.g., '<' or '<=', but provided was '" + label + "'");
}

void QualitativeTimePointConstraint::setType(QualitativeTimePointConstraint::Type type)
{
    mConstraintType = type;
    SimpleConstraint::setLabel(QualitativeTimePointConstraint::TypeTxt[type]);
}


QualitativeTimePointConstraint::Ptr QualitativeTimePointConstraint::create(Variable::Ptr source, Variable::Ptr target, point_algebra::QualitativeTimePointConstraint::Type constraintType)
{
    return make_shared<QualitativeTimePointConstraint>(source, target, constraintType);
}

QualitativeTimePointConstraint::Ptr QualitativeTimePointConstraint::getSymmetric(QualitativeTimePointConstraint::Ptr constraint)
{
    return make_shared<QualitativeTimePointConstraint>(constraint->getTargetVariable(), constraint->getSourceVariable(), getSymmetric(constraint->getType()));
}

QualitativeTimePointConstraint::Type QualitativeTimePointConstraint::getSymmetric(QualitativeTimePointConstraint::Type type)
{
    if(!msInitializedSymmetryTable)
    {
        for(int i = 0; i < (int) TypeEndMarker; ++i)
        {
            std::map<Type, Type>::const_iterator cit = SymmetricType.find((Type) i);
            if(cit == SymmetricType.end())
            {
                throw std::runtime_error("QualitativeTimePointConstraint::getSymmetric: no symmetric type defined");
            }

            msSymmetryTable[i] = cit->second;
        }

        msInitializedSymmetryTable = true;
    }

    return msSymmetryTable[(int) type];
}

QualitativeTimePointConstraint::Type QualitativeTimePointConstraint::getComposition(QualitativeTimePointConstraint::Type firstType, QualitativeTimePointConstraint::Type secondType)
{
    if(firstType == Universal || secondType == Universal)
    {
        return Universal;
    }

    if(!msInitializedCompositionTable)
    {
        for(int a = 0; a < (int) TypeEndMarker; ++a)
        {
            for(int b = 0; b < (int) TypeEndMarker; ++b)
            {
                if(a == b)
                {
                    msCompositionTable[a][b] = (Type) a;
                } else if(a == (int) Empty)
                {
                    msCompositionTable[a][b] = (Type) b;
                } else if(b == (int) Empty)
                {
                    msCompositionTable[a][b] = (Type) a;
                    continue;
                } else if(a == (int) Universal)
                {
                    msCompositionTable[a][b] = Universal;
                } else if(b == (int) Universal)
                {
                    msCompositionTable[a][b] = Universal;
                } else {
                    {
                        std::pair<Type, Type> key((Type) a, (Type) b);
                        std::map< std::pair<Type, Type>, Type >::const_iterator cit = TypeAlgebra.find(key);
                        if(cit != TypeAlgebra.end())
                        {
                            msCompositionTable[a][b] = cit->second;
                            continue;
                        }
                    }
                    {
                        std::pair<Type, Type> key((Type) b, (Type) a);
                        std::map< std::pair<Type, Type>, Type >::const_iterator cit = TypeAlgebra.find(key);
                        if(cit != TypeAlgebra.end())
                        {
                            msCompositionTable[a][b] = getSymmetric(cit->second);
                            continue;
                        }
                    }

                    throw std::runtime_error("QualitativeTimePointConstraint::getComposition found no match for composition of: " + TypeTxt[(QualitativeTimePointConstraint::Type) a] + " and " + TypeTxt[(QualitativeTimePointConstraint::Type) b]);
                }
            }
        }
        msInitializedCompositionTable = true;
    }

    return msCompositionTable[(int) firstType][(int) secondType];
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
    return getIntersection(firstType, secondType) != Empty;
}

QualitativeTimePointConstraint::Type QualitativeTimePointConstraint::getIntersection(QualitativeTimePointConstraint::Type firstType, QualitativeTimePointConstraint::Type secondType)
{
    // This implementation is actually the verbose way of intersecting the set of primitive
    // operators
    if(firstType == Empty || secondType == Empty)
    {
        return Empty;
    } else if(firstType == Universal)
    {
        return secondType;
    } else if(secondType == Universal)
    {
        return firstType;
    }

    if(firstType == secondType)
    {
        return firstType;
    }

    switch(firstType)
    {
        case Less:
            {
                switch(secondType)
                {
                    case Less:
                    case LessOrEqual:
                        return Less;
                    case Distinct:
                        return Less;
                    default:
                        return Empty;
                }
            }
        case Greater:
            {
                switch(secondType)
                {
                    case Greater:
                    case GreaterOrEqual:
                        return Greater;
                    case Distinct:
                        return Greater;
                    default:
                        return Empty;
                }
            }
        case Equal:
            {
                switch(secondType)
                {
                    case GreaterOrEqual:
                    case LessOrEqual:
                    case Equal:
                        return Equal;
                    case Distinct:
                        return Empty;
                    default:
                        return Empty;
                }
            }
        case LessOrEqual:
            {
                switch(secondType)
                {
                    case Less:
                        return Less;
                    case LessOrEqual:
                        return LessOrEqual;
                    case Distinct:
                        return Less;
                    case GreaterOrEqual:
                    case Equal:
                        return Equal;
                    default:
                        return Empty;
                }
            }
        case GreaterOrEqual:
            {
                switch(secondType)
                {
                    case Greater:
                        return Greater;
                    case GreaterOrEqual:
                        return GreaterOrEqual;
                    case Distinct:
                        return Greater;
                    case LessOrEqual:
                    case Equal:
                        return Equal;
                    default:
                        return Empty;
                }
            }
        case Distinct:
            {
                switch(secondType)
                {
                    case GreaterOrEqual:
                    case Greater:
                        return Greater;
                    case LessOrEqual:
                    case Less:
                        return Less;
                    default:
                        return Empty;
                }
            }
        default:
            return Empty;
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

std::string QualitativeTimePointConstraint::toString() const
{
    return SimpleConstraint::toString() + ": " + TypeTxt[getType()];
}

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
