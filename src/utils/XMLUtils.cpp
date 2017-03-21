#include "XMLUtils.hpp"
#include <sstream>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

namespace templ {
namespace utils {

bool XMLUtils::nameMatches(xmlNodePtr node, const std::string& name, bool useNamespace)
{
    std::string nodeName;
    if(node->ns && useNamespace)
    {
        nodeName = std::string( (const char*) node->ns->href);
    }
    nodeName += std::string((const char*) node->name);
    return nodeName == name;
}

std::string XMLUtils::getContent(xmlDocPtr doc, xmlNodePtr node, size_t count)
{
    xmlChar* key = xmlNodeListGetString(doc, node->xmlChildrenNode, count);
    if(key)
    {
        std::string content((const char*) key);
        xmlFree(key);

        std::string http = "http://";
        std::string prefix = content.substr(0,http.size());
        if(prefix != http)
        {
            size_t found = content.find_first_of(':');
            if(found != std::string::npos)
            {
                std::string prefix = content.substr(0,found);
                std::string ns = resolveNamespacePrefix(doc, node, prefix);
                std::string core = content.substr(found+1);
                content = ns + core;
            }
        }
        return content;
    } else {
        return std::string();
    }
}

std::string XMLUtils::resolveNamespacePrefix(xmlDocPtr doc, xmlNodePtr node, const std::string& prefix)
{
    xmlNsPtr *nsList = xmlGetNsList(doc,node);
    xmlNsPtr *deleteRef = nsList;

    std::string href;
    bool found = false;

    // Walk list and register xpath
    while( nsList != NULL && (*nsList)->next)
    {
        if( (*nsList)->prefix != NULL)
        {
            std::string content((const char*) (*nsList)->prefix);
            if(content == prefix && (*nsList)->href != NULL)
            {
                std::string content((const char*) (*nsList)->href);
                href = content;
                found = true;
                break;
            }
        }
        ++nsList; // Next
    }
    if(deleteRef != NULL)
    {
        xmlFree(deleteRef);
    }
    if(found)
    {
        return href;
    } else {
        std::stringstream ss;
        ss << "templ::XMLUtils::resolveNamespacePrefix: could not resolve namespace ";
        ss << "'" << prefix << "' at line: " << xmlGetLineNo(node);
        throw std::invalid_argument(ss.str());
    }
}

std::string XMLUtils::getProperty(xmlNodePtr node, const std::string& name)
{
    std::string property;
    xmlChar* xmlName = xmlCharStrdup(name.c_str());
    xmlChar* value = xmlGetProp(node, xmlName);
    xmlFree(xmlName);
    if(value)
    {
        property = std::string((const char*) value);
        xmlFree(value);
        return property;
    }
    std::stringstream ss;
    ss << "templ::utils::XMLUtils::getProperty: could not find property ";
    ss << "'" << name << "' at line " << xmlGetLineNo(node);
    throw std::invalid_argument(ss.str());
}

std::string XMLUtils::getSubNodeContent(xmlDocPtr doc, xmlNodePtr node, const std::string& name)
{
    xmlNodePtr subNode = node->xmlChildrenNode;
    while(subNode != NULL)
    {
        if(nameMatches(subNode, name))
        {
            return getContent(doc, subNode);
        }

        subNode = subNode->next;
    }
    std::stringstream ss;
    ss << "templ::utils::XMLUtils::getSubNodeContent: could not find subnode ";
    ss << "'" << name << "' in node '" << std::string((const char*) node->name) << "'";
    ss << " at line " << xmlGetLineNo(node);
    throw std::invalid_argument(ss.str());
}

} // end namespace utils
} // end namespace templ
