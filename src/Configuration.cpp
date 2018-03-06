#include "Configuration.hpp"
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <sstream>
#include "utils/XMLUtils.hpp"

namespace templ {

Configuration::Configuration(const std::string& path)
{
    loadXML(path);
}

const std::string& Configuration::getValue(const std::string& key) const
{
    std::map<std::string, std::string>::const_iterator cit = mProperties.find(key);
    if(cit != mProperties.end())
    {
        return cit->second;
    }
    throw std::invalid_argument("templ::Configuration::getValue: key '" + key + "' does not exist");
}

template<>
bool Configuration::getValueAs(const std::string& key, const bool& defaultValue) const
{
    try {
        if(!key.empty())
        {
            std::string value = getValue(key);
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            if(value == "true")
            {
                return true;
            } else if(value == "false")
            {
                return false;
            }
            throw std::runtime_error("templ::Configuration: value for key '" + key + "' cannot be converted to bool, expected one of true or false");
        }
    } catch(const std::invalid_argument& e)
    {
        LOG_DEBUG_S << e.what();
    }

    return defaultValue;
}

void Configuration::loadXML(const std::string& url)
{
    if(url.empty())
    {
        return;
    }

    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    LIBXML_TEST_VERSION

    // The resulting document tree
    xmlDocPtr doc = NULL;

    xmlParserOption options =  XML_PARSE_NOENT; // http://xmlsoft.org/html/libxml-parser.html#xmlParserOption

    try {
        // xmlReadFile can take filename or url
        doc = xmlReadFile(url.c_str(), NULL, options);
        if(doc == NULL)
        {
            throw std::runtime_error("templ::Configuration::loadXML: Failed to parse url '" + url + "'");
        }

        //xmlChar* xpath = (xmlChar*) "//node()[not(node())]";
        xmlChar* xpath = (xmlChar*) "//*[not(*)]";
        xmlXPathContextPtr context;
        xmlXPathObjectPtr result;

        context = xmlXPathNewContext(doc);
        if (context == NULL)
        {
            throw std::runtime_error("templ::Configuration::loadXML: Error in xmlXPathNewContenxt");
        }
        result = xmlXPathEvalExpression(xpath, context);
        xmlXPathFreeContext(context);
        if (result == NULL)
        {
            throw std::runtime_error("templ::Configuration::loadXML: Error in xmlXPathEvalExpression");
        }
        if(xmlXPathNodeSetIsEmpty(result->nodesetval))
        {
           xmlXPathFreeObject(result);
           throw std::runtime_error("templ::Configuration::loadXML: Configuration is empty");
        }

        xmlNodeSetPtr nodeset;
        xmlChar* value;

        if (result)
        {
            nodeset = result->nodesetval;
            for(int i=0; i < nodeset->nodeNr; ++i)
            {
                xmlNode* node = nodeset->nodeTab[i]->xmlChildrenNode;
                value = xmlNodeListGetString(doc,node , 1);
                std::string propertyName = utils::XMLUtils::getFullPath(doc, node);
                if( mProperties.count(propertyName) )
                {
                    throw std::runtime_error("templ::Configuration::loadXML: property '" + propertyName + "' with multiple entries (while it must have unique entries)");
                } else {
                    mProperties[propertyName] = std::string( (char*) value );
                    LOG_INFO_S << "Add configuration: " << propertyName << " " << value;
                }
                xmlFree(value);
            }
            xmlXPathFreeObject (result);
        }

    } catch(const std::exception& e)
    {
        xmlFreeDoc(doc);
        xmlCleanupParser();
        throw;
    }

    /*
     * Cleanup function for the XML library.
     */
    xmlCleanupParser();

}

std::string Configuration::toString() const
{
    std::stringstream ss;
    for(const std::pair<std::string, std::string>& p : mProperties)
    {
        ss << p.first << " -> " << p.second << std::endl;
    }
    return ss.str();
}

} // end namespace templ
