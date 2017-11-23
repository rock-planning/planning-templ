#include "XMLTCNUtils.hpp"
#include <base-logging/Logging.hpp>

namespace templ {
namespace utils {

std::vector<templ::io::TemporalConstraint> XMLTCNUtils::parseTemporalConstraints(xmlDocPtr doc, xmlNodePtr current)
{
    using namespace templ::io;

    std::vector<templ::io::TemporalConstraint> constraints;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(! (XMLUtils::nameMatches(current,"text") || XMLUtils::nameMatches(current, "comment")))
        {
            TemporalConstraint constraint;
            constraint.type = TemporalConstraint::getTemporalConstraintType( std::string((const char*) current->name) );
            constraint.lval = XMLUtils::getProperty(current, "lval");
            constraint.rval = XMLUtils::getProperty(current, "rval");

            LOG_DEBUG_S << "Parsed temporal constraint: " << constraint.toString();
            constraints.push_back(constraint);
        }

        current = current->next;
    }
    return constraints;
}

templ::io::TemporalRequirement XMLTCNUtils::parseTemporalRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    using namespace templ::io;

    TemporalRequirement requirement;

    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(XMLUtils::nameMatches(current,"from"))
        {
            requirement.from = XMLUtils::getContent(doc, current);
        } else if(XMLUtils::nameMatches(current, "to"))
        {
            requirement.to = XMLUtils::getContent(doc, current);
        }
        current = current->next;
    }
    return requirement;
}

templ::solvers::temporal::TemporalConstraintNetwork::Ptr XMLTCNUtils::readTemporalConstraintNetwork(xmlDocPtr doc, xmlNodePtr current)
{
    using namespace templ::solvers::temporal;
    using namespace templ::io;

    TemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    std::vector<TemporalConstraint> temporalConstraints = parseTemporalConstraints(doc, current);

    std::vector<TemporalConstraint>::const_iterator cit = temporalConstraints.begin();
    for(; cit != temporalConstraints.end(); ++cit)
    {
        TemporalConstraint tc = *cit;

        point_algebra::TimePoint::Ptr lval = tcn->getOrCreateTimePoint(tc.lval);
        point_algebra::TimePoint::Ptr rval = tcn->getOrCreateTimePoint(tc.rval);

        tcn->addQualitativeConstraint(lval, rval, tc.type);
    }
    return tcn;
}

} // end namespace utils
} // end namespace templ
