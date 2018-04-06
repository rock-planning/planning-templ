#ifndef TEMPL_UTILS_XMLTCN_UTILS_HPP
#define TEMPL_UTILS_XMLTCN_UTILS_HPP

#include <vector>
#include <qxcfg/utils/XMLUtils.hpp>
#include "../io/MissionRequirements.hpp"
#include "../solvers/temporal/TemporalConstraintNetwork.hpp"

namespace templ {
namespace utils {

class XMLTCNUtils
{
public:
    /**
     * Parse section of temporal constraints
     */
    static std::vector<templ::io::TemporalConstraint> parseTemporalConstraints(xmlDocPtr doc, xmlNodePtr current);

    static templ::io::TemporalRequirement parseTemporalRequirement(xmlDocPtr doc, xmlNodePtr current);

    static templ::solvers::temporal::TemporalConstraintNetwork::Ptr readTemporalConstraintNetwork(xmlDocPtr doc, xmlNodePtr current);

};

} // end namespace utils
} // end namespae templ

#endif // TEMPL_UTILS_XMLTCN_UTILS_HPP

