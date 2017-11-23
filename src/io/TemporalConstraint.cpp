#include "TemporalConstraint.hpp"
#include <sstream>

namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace io {

pa::QualitativeTimePointConstraint::Type TemporalConstraint::getTemporalConstraintType(const std::string& name)
{
    using namespace templ::solvers::temporal::point_algebra;
    //return Empty, Greater, Less, Equal, Distinct, GreaterOrEqual, LessOrEqual, Universal,
    if(name == "greaterThan")
    {
        return QualitativeTimePointConstraint::Greater;
    } else if(name == "lessThan")
    {
        return QualitativeTimePointConstraint::Less;
    } else if(name == "equals")
    {
        return QualitativeTimePointConstraint::Equal;
    } else if(name == "distinct")
    {
        return QualitativeTimePointConstraint::Distinct;
    } else if(name == "greaterOrEqual")
    {
        return QualitativeTimePointConstraint::GreaterOrEqual;
    } else if(name == "lessOrEqual")
    {
        return QualitativeTimePointConstraint::LessOrEqual;
    }

    throw std::invalid_argument("templ::io::MissionReader::getTemporalConstraintType: unknown temporal constraint type: '" + name + "'");
}

std::string TemporalConstraint::toXML(pa::QualitativeTimePointConstraint::Type type)
{
    using namespace templ::solvers::temporal::point_algebra;
    switch(type)
    {
        case QualitativeTimePointConstraint::Greater:
            return "greaterThan";
        case QualitativeTimePointConstraint::Less:
            return "lessThan";
        case QualitativeTimePointConstraint::Equal:
            return "equals";
        case QualitativeTimePointConstraint::Distinct:
            return "distinct";
        case QualitativeTimePointConstraint::GreaterOrEqual:
            return "greaterOrEqual";
        case QualitativeTimePointConstraint::LessOrEqual:
            return "lessOrEqual";
        default:
            break;
        //case QualitativeTimePointConstraint::Universal:
        //    return "universal";
        //case QualitativeTimePointConstraint::Empty:
        //    return "empty";
    }
    throw std::invalid_argument("templ::io::MissionReader::toXML: unknown xml conversion for temporal constraint type: '" + pa::QualitativeTimePointConstraint::TypeTxt[type] + "'");
}

std::string TemporalConstraint::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "TemporalConstraint:" << std::endl;
    ss << hspace << "    type:" << templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::TypeTxt[type];
    ss << hspace << "    lval:" << lval;
    ss << hspace << "    rval:" << rval;
    return ss.str();
}

} // end namespace io
} // end namespace templ
