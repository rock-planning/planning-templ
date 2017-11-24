#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>

namespace templ {
namespace io {

struct TemporalConstraint
{
    templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type type;

    /// Use for rval and to
    std::string rval;
    /// Use for lval and from
    std::string lval;

    double minDuration;
    double maxDuration;

    TemporalConstraint();

    // Convert from an XML label to the corresponding xml type
    static templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type getTemporalConstraintType(const std::string& name);
    // Convert the constraint type to an XML label
    static std::string toXML(templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type type);

    std::string toString(uint32_t indent = 0) const;
};

} // end namespace io
} // end namespace templ

