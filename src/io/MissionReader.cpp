#include <templ/io/MissionReader.hpp>

#include <sstream>
#include <boost/lexical_cast.hpp>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <base/Logging.hpp>

namespace templ {
namespace io {

std::string Location::toString() const
{
    std::stringstream ss;
    ss << "Location: " << id;
    return ss.str();
}


std::string SpatialRequirement::toString() const
{
    std::stringstream ss;
    ss << "SpatialRequirement:" << std::endl;
    ss << "    location: " << location.toString() << std::endl;
    return ss.str();
}

std::string TemporalRequirement::toString() const
{
    std::stringstream ss;
    ss << "TemporalRequirement:" << std::endl;
    ss << "    id:   " << id << std::endl;
    ss << "    from: " << from << std::endl;
    ss << "    to:   " << to << std::endl;
    return ss.str();
}


std::string ServiceRequirement::toString() const
{
    return owlapi::model::IRI::toString(services);
}


std::string Requirement::toString() const
{
    std::stringstream ss;
    ss << "Requirement: " << std::endl;
    ss << spatial.toString();
    ss << temporal.toString();
    ss << "Services: ";
    ss << services.toString() << std::endl;
    ss << resources.toString();
    return ss.str();
}

templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type TemporalConstraint::getTemporalConstraintType(const std::string& name)
{
    using namespace templ::solvers::temporal::point_algebra;
    //return Empty, Greater, Less, Equal, Distinct, GreaterOrEqual, LessOrEqual, Universal,
    if(name == "greaterThan")
    {
        return QualitativeTimePointConstraint::Greater;
    } else if(name == "lessThan")
    {
        return QualitativeTimePointConstraint::Less;
    } else if(name == "equals")
    {
        return QualitativeTimePointConstraint::Equal;
    } else if(name == "distinct")
    {
        return QualitativeTimePointConstraint::Distinct;
    } else if(name == "greaterOrEqual")
    {
        return QualitativeTimePointConstraint::GreaterOrEqual;
    } else if(name == "lessOrEqual")
    {
        return QualitativeTimePointConstraint::LessOrEqual;
    }

    throw std::invalid_argument("templ::MissionReader: unknown temporal constraint type: '" + name + "'");
}

std::string TemporalConstraint::toString() const
{
    std::stringstream ss;
    ss << "TemporalConstraint:" << std::endl;
    ss << "    type:" << templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::TypeTxt[type];
    ss << "    lval:" << lval;
    ss << "    rval:" << rval;
    return ss.str();
}

bool MissionReader::nameMatches(xmlNodePtr node, const std::string& name)
{
    return xmlStrcmp(node->name, (const xmlChar*) name.c_str()) == 0;
}

std::string MissionReader::getContent(xmlDocPtr doc, xmlNodePtr node, size_t count)
{
    xmlChar* key = xmlNodeListGetString(doc, node->xmlChildrenNode, count);
    if(key)
    {
        std::string content((const char*) key);
        xmlFree(key);

        return content;
    } else {
        return std::string();
    }
}

std::string MissionReader::getProperty(xmlNodePtr node, const std::string& name)
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
    throw std::invalid_argument("templ::io::MissionReader::getProperty: could not find property '" + name + "'");
}

std::string MissionReader::getSubNodeContent(xmlDocPtr doc, xmlNodePtr node, const std::string& name)
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
    throw std::invalid_argument("templ::io::MissionReader::getSubNodeContent: could not find subnode '" + name + "'");
}

Mission MissionReader::fromFile(const std::string& url)
{
    Mission mission;

    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    LIBXML_TEST_VERSION

    // The resulting document tree
    xmlDocPtr doc;

    xmlParserOption options = (xmlParserOption) 0;

    try {
        // xmlReadFile can take filename or url
        doc = xmlReadFile(url.c_str(), NULL, options);
        if(doc == NULL)
        {
            throw std::runtime_error("Failed to parse url '" + url + "'");
        }

        xmlNodePtr rootNode;
        rootNode = xmlDocGetRootElement(doc);
        if(rootNode == NULL)
        {
            throw std::runtime_error("Empty document");
        }

        if(xmlStrcmp(rootNode->name, (const xmlChar*) "mission"))
        {
            throw std::runtime_error("Unexpected root node type: '" + std::string((const char*) rootNode->name) );
        }
        LOG_INFO_S << "Found root node: " << rootNode->name;

        xmlNodePtr firstLevelChild = rootNode->xmlChildrenNode;
        while(firstLevelChild != NULL)
        {
            if(nameMatches(firstLevelChild, "name"))
            {
                LOG_DEBUG_S << "Found first level node: 'name' " << getContent(doc, firstLevelChild);
            } else if(nameMatches(firstLevelChild, "resources"))
            {
                LOG_DEBUG_S << "Found first level node: 'resources' ";
                parseResources(doc, firstLevelChild);
            } else if(nameMatches(firstLevelChild, "requirements"))
            {
                LOG_DEBUG_S << "Found first level node: 'requirements' ";
                parseRequirements(doc, firstLevelChild);
            } else if(nameMatches(firstLevelChild, "constraints"))
            {
                LOG_DEBUG_S << "Found first level node: 'constraints' ";
                parseConstraints(doc, firstLevelChild);
            }
            firstLevelChild = firstLevelChild->next;
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

    return mission;
}

void MissionReader::parseResource(xmlDocPtr doc, xmlNodePtr current)
{
    if(nameMatches(current, "resource"))
    {
        LOG_INFO_S << "Parsing: " << current->name;
    }
}

void MissionReader::parseResources(xmlDocPtr doc, xmlNodePtr current)
{
    LOG_INFO_S << "Parsing: " << current->name;
    xmlNodePtr resource = current->xmlChildrenNode;
    while(resource != NULL)
    {
        parseResource(doc, resource);
        resource = resource->next;
    }
}

SpatialRequirement MissionReader::parseSpatialRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    SpatialRequirement requirement;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "location"))
        {
            requirement.location.id = getSubNodeContent(doc, current, "id");
        }
        current = current->next;
    }
    return requirement;
}

TemporalRequirement MissionReader::parseTemporalRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    TemporalRequirement requirement;

    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "quantitative"))
        {
            std::string from = getSubNodeContent(doc, current, "from");
            requirement.from = boost::lexical_cast<uint32_t>(from);

            std::string to = getSubNodeContent(doc, current, "to");
            if(to == "inf")
            {
                requirement.to = std::numeric_limits<uint32_t>::infinity();
            } else {
                requirement.to = boost::lexical_cast<uint32_t>(to);
            }
        } else if(nameMatches(current, "qualitative"))
        {
            requirement.id = getSubNodeContent(doc, current, "id");
        }
        current = current->next;
    }
    return requirement;
}

ServiceRequirement MissionReader::parseServiceRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    ServiceRequirement requirement;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "service"))
        {
            std::string service = getContent(doc, current);
            requirement.services.push_back( owlapi::model::IRI(service) );
        }
        current = current->next;
    }
    return requirement;
}

organization_model::ModelPool MissionReader::parseResourceRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    organization_model::ModelPool modelPool;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "resource"))
        {
            std::string model = getSubNodeContent(doc, current, "model");
            std::string minCardinalityTxt = getSubNodeContent(doc, current, "minCardinality");

            uint32_t minCardinality = boost::lexical_cast<uint32_t>(minCardinalityTxt);
            modelPool[ owlapi::model::IRI(model) ] = minCardinality;
        }
        current = current->next;
    }

    return modelPool;
}

Requirement MissionReader::parseRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    Requirement requirement;
    if(nameMatches(current, "requirement"))
    {
        LOG_INFO_S << "Parsing: " << current->name;

        xmlNodePtr requirementNode = current->xmlChildrenNode;
        while(requirementNode != NULL)
        {
            if(nameMatches(requirementNode, "spatial-requirement"))
            {
                LOG_DEBUG_S << "Parse spatial requirement";
                requirement.spatial = parseSpatialRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed spatial requirement: " << requirement.spatial.toString();

            } else if(nameMatches(requirementNode, "temporal-requirement"))
            {
                LOG_DEBUG_S << "Parse temporal requirement";
                requirement.temporal = parseTemporalRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed temporal requirement: " << requirement.temporal.toString();
            } else if(nameMatches(requirementNode, "service-requirement"))
            {
                LOG_DEBUG_S << "Parse service requirement";
                requirement.services = parseServiceRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed service requirement: " << requirement.services.toString();
            } else if(nameMatches(requirementNode, "resource-requirement"))
            {
                LOG_DEBUG_S << "Parse resource requirement";
                requirement.resources = parseResourceRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed resource requirement: " << requirement.resources.toString();
            }
            requirementNode = requirementNode->next;
        }
    } else {
        throw std::invalid_argument("Unexpected tag: '" + std::string((const char*) current->name) + "' expected requirement");
    }
    return requirement;
}

void MissionReader::parseRequirements(xmlDocPtr doc, xmlNodePtr current)
{
    LOG_INFO_S << "Parsing: " << current->name;
    xmlNodePtr requirement = current->xmlChildrenNode;
    while(requirement != NULL)
    {
        try {
            Requirement r = parseRequirement(doc, requirement);
            LOG_INFO_S << "Parsed requirement: " << r.toString();
        } catch(const std::invalid_argument& e)
        {
            LOG_WARN_S << e.what();
            // filter out 'text' nodes
        }
        requirement = requirement->next;
    }
}

std::vector<TemporalConstraint> MissionReader::parseTemporalConstraints(xmlDocPtr doc, xmlNodePtr current)
{
    std::vector<TemporalConstraint> constraints;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(!nameMatches(current,"text"))
        {
            TemporalConstraint constraint;
            constraint.type = TemporalConstraint::getTemporalConstraintType( std::string((const char*) current->name) );
            constraint.lval = getProperty(current, "lval");
            constraint.rval = getProperty(current, "rval");

            LOG_DEBUG_S << "Parsed temporal constraint: " << constraint.toString();
            constraints.push_back(constraint);
        }

        current = current->next;
    }
    return constraints;
}

Constraints MissionReader::parseConstraints(xmlDocPtr doc, xmlNodePtr current)
{
    LOG_INFO_S << "Parsing: " << current->name;
    Constraints constraints;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "temporal-constraints"))
        {
            LOG_INFO_S << "Parsing: " << current->name;
            constraints.temporal = parseTemporalConstraints(doc, current);
        }
        current = current->next;
    }
    return constraints;
}

} // end namespace io
} // end namespace templ

