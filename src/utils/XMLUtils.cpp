#include "XMLUtils.hpp"
#include <sstream>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

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

std::vector<templ::io::TemporalConstraint> XMLUtils::parseTemporalConstraints(xmlDocPtr doc, xmlNodePtr current)
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

templ::io::TemporalRequirement XMLUtils::parseTemporalRequirement(xmlDocPtr doc, xmlNodePtr current)
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

templ::io::Constraints XMLUtils::parseConstraints(xmlDocPtr doc, xmlNodePtr current)
{
    LOG_DEBUG_S << "Parsing: " << current->name;
    templ::io::Constraints constraints;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(XMLUtils::nameMatches(current, "temporal-constraints"))
        {
            LOG_DEBUG_S << "Parsing: " << current->name;
            constraints.temporal = XMLUtils::parseTemporalConstraints(doc, current);
        }
        current = current->next;
    }
    return constraints;
}

templ::solvers::temporal::TemporalConstraintNetwork::Ptr XMLUtils::readTemporalConstraintNetwork(xmlDocPtr doc, xmlNodePtr current)
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
