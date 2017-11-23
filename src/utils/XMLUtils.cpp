#include "XMLUtils.hpp"
#include <sstream>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <base-logging/Logging.hpp>

namespace templ {
namespace utils {

std::string XMLUtils::getName(xmlNodePtr node, bool useNamespace)
{
    std::string nodeName;
    if(node->ns && useNamespace)
    {
        nodeName = std::string( (const char*) node->ns->href);
    }
    nodeName += std::string((const char*) node->name);
    return nodeName;
}

bool XMLUtils::nameMatches(xmlNodePtr node, const std::string& name, bool useNamespace)
{
    return getName(node, useNamespace) == name;
}

bool XMLUtils::hasContent(xmlDocPtr doc, xmlNodePtr node, size_t count)
{
    xmlChar* key = xmlNodeListGetString(doc, node->xmlChildrenNode, count);
    if(key)
    {
        xmlFree(key);
        return true;
    }
    return false;
}

std::string XMLUtils::getContent(xmlDocPtr doc, xmlNodePtr node, size_t count, bool plainText)
{
    xmlChar* key = xmlNodeListGetString(doc, node->xmlChildrenNode, count);
    if(key)
    {
        std::string content((const char*) key);
        xmlFree(key);

        if(!plainText)
        {
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

std::string XMLUtils::getFullPath(xmlDocPtr doc, xmlNodePtr node, const std::string& separator, bool includeRoot)
{
    std::string s;
    xmlNodePtr root = xmlDocGetRootElement(doc);

    xmlNodePtr currentNode = node->parent;
    while(currentNode->parent)
    {
        if(s.empty())
        {
            s = getName(currentNode, false);
        } else {
            s = getName(currentNode,false) + separator + s;
        }
        if(!includeRoot && currentNode->parent == root)
        {
            break;
        }
        currentNode = currentNode->parent;
    }
    return s;
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

void XMLUtils::writeComment(xmlTextWriterPtr writer, const std::string& comment, const std::string& encoding)
{
    xmlChar* xmlComment = convertInput(comment.c_str(), encoding.c_str());
    int rc = xmlTextWriterWriteComment(writer, xmlComment);
    if(rc < 0)
    {
        throw std::runtime_error("templ::utils::XMLUtils::write: failed to"
                " write comment: '" + comment + "'");
    }
    if(xmlComment != NULL)
    {
        xmlFree(xmlComment);
    }
}

void XMLUtils::writeCDATA(xmlTextWriterPtr writer, const std::string& cdata, const std::string& encoding)
{
    TEMPL_XML_RESULT_CHECK( xmlTextWriterWriteCDATA(writer, convertInput( cdata.c_str(), encoding.c_str() ) ) , writeCDATA);
}

void XMLUtils::writeString(xmlTextWriterPtr writer, const std::string& cdata, const std::string& encoding)
{
    TEMPL_XML_RESULT_CHECK( xmlTextWriterWriteString(writer, convertInput( cdata.c_str(), encoding.c_str() ) ), writeString );
}

void XMLUtils::startElement(xmlTextWriterPtr writer, const std::string& element)
{
    int rc = xmlTextWriterStartElement(writer, BAD_CAST element.c_str());
    if (rc < 0)
    {
        throw std::runtime_error("templ::solvers::utils::XMLUtils::startElement failed"
                " for element '" + element + "'");
    }
}

void XMLUtils::endElement(xmlTextWriterPtr writer)
{
    int rc = xmlTextWriterEndElement(writer);
    if (rc < 0)
    {
        throw std::runtime_error("templ::solvers::utils::XMLUtils::endElement failed");
    }
}


xmlChar* XMLUtils::convertInput(const char *in, const char *encoding)
{
    xmlChar* out;
    int ret;
    int size;
    int out_size;
    int temp;
    xmlCharEncodingHandlerPtr handler;

    if (in == 0)
    {
        return NULL;
    }

    handler = xmlFindCharEncodingHandler(encoding);

    if (!handler)
    {
        throw std::runtime_error("templ::utils::XMLUtils: no encoding handler found for" + std::string(encoding));
    }

    size = (int) strlen(in) + 1;
    out_size = size * 2 - 1;
    out = (unsigned char *) xmlMalloc((size_t) out_size);

    if (out != 0) {
        temp = size - 1;
        ret = handler->input(out, &out_size, (const xmlChar *) in, &temp);
        if ((ret < 0) || (temp - size + 1)) {
            if (ret < 0)
            {
                LOG_WARN_S << "conversion wasn't successful";
            }
            xmlFree(out);
            out = 0;
        } else {
            out = (unsigned char *) xmlRealloc(out, out_size + 1);
            out[out_size] = 0;  /*null terminating out */
        }
    } else {
        LOG_WARN_S << "no memory";
    }

    return out;
}

void XMLUtils::lint(const std::string& path)
{
    // the formatting stage
    std::string formattedFile = path + ".formatted";
    std::string command = "`which xmllint` --encode UTF-8 --format " + path + " > " + formattedFile;

    LOG_INFO("Trying to format using xmllint: '%s'", command.c_str());
    if( system(command.c_str()) == 0 )
    {
        command = "mv " + formattedFile + " " + path;
        if( system(command.c_str()) )
        {
            throw std::runtime_error("templ::utils::XMLUtils::lint: Failed to rename file after performing xmllint");
        }
    } else {
        LOG_WARN("XML file '%s' written, but proper formatting failed -- make sure that xmllint is installed", path.c_str());
    }
}


} // end namespace utils
} // end namespace templ
