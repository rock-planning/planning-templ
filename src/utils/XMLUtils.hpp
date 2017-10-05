#ifndef TEMPL_UTILS_XML_UTILS_HPP
#define TEMPL_UTILS_XML_UTILS_HPP

#include <string>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <boost/lexical_cast.hpp>
#include "../io/MissionRequirements.hpp"
#include "../solvers/temporal/TemporalConstraintNetwork.hpp"

namespace templ {
namespace utils {

/**
 * \class XMLUtils
 * \brief Provide a set of helper functions to parse XML files and extract
 * information from xml nodes
 */
class XMLUtils
{
public:
    /**
     * Get name of the node
     * \param node xml node to retrieve the name from
     * \param useNamespace whether to prepend the namespace or not
     * \return name
     */
    static std::string getName(xmlNodePtr node, bool useNamespace = false);

    /**
     * Check if the name of the node matches the given string
     * \param node xml node to retrieve the name from
     * \param name name to match against the name of the given node
     * \param useNamespace whether to prepend the namespace or not
     * \return true if name matches, false otherwise
     */
    static bool nameMatches(xmlNodePtr node, const std::string& name, bool useNamespace = false);

    static std::string resolveNamespacePrefix(xmlDocPtr doc, xmlNodePtr node, const std::string& prefix);

    static std::string getContent(xmlDocPtr doc, xmlNodePtr node, size_t count = 1);

    static bool hasContent(xmlDocPtr doc, xmlNodePtr node, size_t count = 1);

    static std::string getProperty(xmlNodePtr node, const std::string& name);

    static std::string getFullPath(xmlDocPtr doc, xmlNodePtr node, const std::string& separator = "/", bool includeRoot = false);

    template <typename T>
    static T getNumericProperty(xmlNodePtr node, const std::string& name)
    {
        std::string propertyTxt = getProperty(node, name);
        return boost::lexical_cast<T>(propertyTxt);
    }

    /**
     * Retrieve the content of a named subnode
     * \param doc xml document
     * \param node the current node
     * \param name the name of the subnode
     * \throws std::invalid_argument if subnode of given name cannot be found
     */
    static std::string getSubNodeContent(xmlDocPtr doc, xmlNodePtr node, const std::string& name);

    /**
     * Parse section of temporal constraints
     */
    static std::vector<templ::io::TemporalConstraint> parseTemporalConstraints(xmlDocPtr doc, xmlNodePtr current);

    static templ::io::TemporalRequirement parseTemporalRequirement(xmlDocPtr doc, xmlNodePtr current);

    static templ::io::Constraints parseConstraints(xmlDocPtr doc, xmlNodePtr current);

    static templ::solvers::temporal::TemporalConstraintNetwork::Ptr readTemporalConstraintNetwork(xmlDocPtr doc, xmlNodePtr current);
};

} // end namespace utils
} // end namespace templ
#endif // TEMPL_UTILS_XML_UTILS_HPP
