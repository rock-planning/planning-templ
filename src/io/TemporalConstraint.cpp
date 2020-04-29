#include "TemporalConstraint.hpp"
#include <sstream>

namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace io {

TemporalConstraint::TemporalConstraint()
    : minDuration(0)
    , maxDuration(std::numeric_limits<double>::max())
{}

pa::QualitativeTimePointConstraint::Type TemporalConstraint::getTemporalConstraintType(const std::string& name)
{
    return pa::QualitativeTimePointConstraint::getTypeFromWord(name);
}

std::string TemporalConstraint::toXML(pa::QualitativeTimePointConstraint::Type type)
{
    return pa::QualitativeTimePointConstraint::TypeWord[type];
}

std::string TemporalConstraint::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "TemporalConstraint:" << std::endl;
    ss << hspace << "    type:" <<
        pa::QualitativeTimePointConstraint::TypeWord[type];
    ss << hspace << "    lval:" << lval;
    ss << hspace << "    rval:" << rval;
    return ss.str();
}

} // end namespace io
} // end namespace templ
