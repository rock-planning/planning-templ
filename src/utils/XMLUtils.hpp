#ifndef TEMPL_UTILS_XML_UTILS_HPP
#define TEMPL_UTILS_XML_UTILS_HPP

#include <string>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <boost/lexical_cast.hpp>

namespace templ {
namespace utils {

class XMLUtils
{
public:
    static bool nameMatches(xmlNodePtr node, const std::string& name, bool useNamespace = false);

    static std::string resolveNamespacePrefix(xmlDocPtr doc, xmlNodePtr node, const std::string& prefix);

    static std::string getContent(xmlDocPtr doc, xmlNodePtr node, size_t count = 1);

    static std::string getProperty(xmlNodePtr node, const std::string& name);

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
     * \param the name of the subnode
     * \throws std::invalid_argument if subnode of given name cannot be found
     */
    static std::string getSubNodeContent(xmlDocPtr doc, xmlNodePtr node, const std::string& name);

};

} // end namespace utils
} // end namespace templ
#endif // TEMPL_UTILS_XML_UTILS_HPP
