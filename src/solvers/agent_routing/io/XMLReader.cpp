#include "XMLReader.hpp"
#include <libxml/encoding.h>
#include <base-logging/Logging.hpp>
#include <sstream>
#include <boost/lexical_cast.hpp>

using namespace templ::utils;
namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace solvers {
namespace agent_routing {
namespace io {

XMLReader::XMLReader()
    : Reader()
{}

XMLReader::~XMLReader()
{}

void XMLReader::read(const std::string& url, AgentRoutingProblem& arp)
{
    mAgentRoutingProblem = AgentRoutingProblem();

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
            throw std::runtime_error("templ::agent_routing::io::XMLReader::fromFile: Failed to parse url '" + url + "'");
        }

        xmlNodePtr rootNode;
        rootNode = xmlDocGetRootElement(doc);
        if(rootNode == NULL)
        {
            throw std::invalid_argument("templ::agent_routing::io::XMLReader::fromFile: Empty document");
        }

        if(xmlStrcmp(rootNode->name, (const xmlChar*) "agent-routing-problem"))
        {
            throw std::invalid_argument("templ::agent_routing::io::XMLReader::fromFile: Unexpected root node type: '" + std::string((const char*) rootNode->name) + "' -- expected <agent-routing-problem> tag");
        }
        LOG_DEBUG_S << "Found root node: " << rootNode->name;

        xmlNodePtr firstLevelChild = rootNode->xmlChildrenNode;
        while(firstLevelChild != NULL)
        {
            if(XMLUtils::nameMatches(firstLevelChild, "agent-types"))
            {
                LOG_DEBUG_S << "Found first level node: 'agent-types";
                readAgentTypes(doc, firstLevelChild);
            } else if(XMLUtils::nameMatches(firstLevelChild, "agents"))
            {
                LOG_DEBUG_S << "Found first level node: 'agents'";
                readAgents(doc, firstLevelChild);
            } else if(XMLUtils::nameMatches(firstLevelChild, "agent-attributes"))
            {
                LOG_DEBUG_S << "Found first level node: 'agent-attributes";
                readAgentAttributes(doc, firstLevelChild);

            } else if(XMLUtils::nameMatches(firstLevelChild, "temporal-constraint-network") )
            {
                LOG_DEBUG_S << "Found first level node: 'temporal-constraint-network'";

                solvers::temporal::TemporalConstraintNetwork::Ptr tcn = XMLTCNUtils::readTemporalConstraintNetwork(doc, firstLevelChild);
                mAgentRoutingProblem.setTemporalConstraintNetwork(tcn);
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

    arp = mAgentRoutingProblem;
    LOG_DEBUG_S << "Read problem: " << arp.toString();
}


void XMLReader::readAgentTypes(xmlDocPtr doc, xmlNodePtr current)
{
    if(XMLUtils::nameMatches(current, "agent-types"))
    {
        current = current->xmlChildrenNode;
        while(current != NULL)
        {
            if(XMLUtils::nameMatches(current, "agent-type"))
            {
                AgentType agentType = parseAgentType(doc, current);
                mAgentRoutingProblem.addAgentType(agentType);
                LOG_DEBUG_S << "Read agent type: " << agentType.toString();
            }
            current = current->next;
        }
    }
}

AgentType XMLReader::parseAgentType(xmlDocPtr doc, xmlNodePtr current)
{
    if( XMLUtils::nameMatches(current, "agent-type") )
    {
        LOG_DEBUG_S << "Parsing: " << current->name;
        std::string idTxt = XMLUtils::getProperty(current, "id");
        uint32_t id = boost::lexical_cast<uint32_t>(idTxt);
        AgentType agentType(id);

        xmlNodePtr subNode = current->xmlChildrenNode;
        while(subNode != NULL)
        {
            if(XMLUtils::nameMatches(subNode, "attribute"))
            {
                AgentIntegerAttribute integerAttribute = parseIntegerAttribute(doc, subNode);
                agentType.addIntegerAttribute( integerAttribute );
            }
            subNode = subNode->next;
        }
        return agentType;
    } else {
        throw std::invalid_argument("templ::agent_routing::io::XMLReader::fromFile: Unexpected root node type: '" + std::string((const char*) current->name) + "' -- expected <agent-type> tag");
    }
}

AgentIntegerAttribute XMLReader::parseIntegerAttribute(xmlDocPtr doc, xmlNodePtr current)
{
    if(XMLUtils::nameMatches(current, "attribute") || XMLUtils::nameMatches(current, "integer-attribute"))
    {
        std::string idTxt = XMLUtils::getProperty(current, "id");
        uint32_t id = boost::lexical_cast<uint32_t>(idTxt);

        AgentIntegerAttribute attribute(id);

        std::string valueTxt = XMLUtils::getContent(doc, current);
        if(!valueTxt.empty())
        {
            uint32_t value = boost::lexical_cast<uint32_t>(valueTxt);
            attribute.setValue(value);
        }

        LOG_DEBUG_S << "Parsed attribute: " << attribute.toString();
        return attribute;
    } else {
        throw std::invalid_argument("templ::agent_routing::io::XMLReader::fromFile: Unexpected root node type: '" + std::string((const char*) current->name) + "' -- expected <attribute> tag");
    }
}

void XMLReader::readAgents(xmlDocPtr doc, xmlNodePtr current)
{

    if(XMLUtils::nameMatches(current, "agents"))
    {
        current = current->xmlChildrenNode;
        while(current != NULL)
        {
            if(XMLUtils::nameMatches(current, "agent"))
            {
                Agent agent = parseAgent(doc, current);
                mAgentRoutingProblem.addAgent(agent);
            }
            current = current->next;
        }
    } else {
        throw std::invalid_argument("templ::agent_routing::io::XMLReader::fromFile: Unexpected root node type: '" + std::string((const char*) current->name) + "' -- expected <agents> tag");
    }
}

Agent XMLReader::parseAgent(xmlDocPtr doc, xmlNodePtr current)
{
    if(XMLUtils::nameMatches(current, "agent"))
    {
        LOG_DEBUG_S << "Parsing: " << current->name;
        std::string idTxt = XMLUtils::getProperty(current, "id");
        uint32_t id = boost::lexical_cast<uint32_t>(idTxt);

        std::string typeTxt = XMLUtils::getProperty(current, "type");
        uint32_t type = boost::lexical_cast<uint32_t>(typeTxt);

        Agent agent(id, type);

        xmlNodePtr subNode = current->xmlChildrenNode;
        while(subNode != NULL)
        {
            if(XMLUtils::nameMatches(subNode, "tasks"))
            {
                LOG_DEBUG_S << "Parsing: tasks";
                AgentTask::List tasks = parseTasks(doc, subNode);
                AgentTask::List::const_iterator cit = tasks.begin();
                for(; cit != tasks.end(); ++cit)
                {
                    agent.addTask(*cit);
                }
            }
            subNode = subNode->next;
        }
        return agent;
    } else {
        throw std::invalid_argument("templ::agent_routing::io::XMLReader::fromFile: Unexpected root node type: '" + std::string((const char*) current->name) + "' -- expected <agent> tag");
    }
}


AgentTask XMLReader::parseTask(xmlDocPtr doc, xmlNodePtr current)
{
    if(XMLUtils::nameMatches(current, "task"))
    {
        uint32_t priority = XMLUtils::getNumericProperty<uint32_t>(current, "priority");
        uint32_t duration = XMLUtils::getNumericProperty<uint32_t>(current, "duration");

        AgentTask task;
        task.setTaskPriority(priority);
        task.setTaskDuration(duration);

        xmlNodePtr subNode = current->xmlChildrenNode;
        while(subNode != NULL)
        {
            if(XMLUtils::nameMatches(subNode, "location"))
            {
                double x = XMLUtils::getNumericProperty<double>(subNode, "x");
                double y = XMLUtils::getNumericProperty<double>(subNode, "y");

                std::string label = XMLUtils::getContent(doc, subNode);
                base::Point p;
                p.x() = x;
                p.y() = y;
                symbols::constants::Location location(label, p );
                task.setLocation(location);

            } else if(XMLUtils::nameMatches(subNode, "arrival"))
            {
                std::string arrivalTime = XMLUtils::getContent(doc, subNode);

                task.setArrival(pa::TimePoint::Ptr( new pa::QualitativeTimePoint(arrivalTime) ) );
            } else if(XMLUtils::nameMatches(subNode, "departure"))
            {
                std::string departureTime = XMLUtils::getContent(doc, subNode);
                task.setDeparture(pa::TimePoint::Ptr( new pa::QualitativeTimePoint(departureTime) ) );
            }

            subNode = subNode->next;
        }

        return task;
    }

    throw std::invalid_argument("templ::agent_routing::io::XMLReader::fromFile: Unexpected root node type: '" + std::string((const char*) current->name) + "' -- expected <tasks> tag");
}


AgentTask::List XMLReader::parseTasks(xmlDocPtr doc, xmlNodePtr current)
{
    AgentTask::List list;
    if( XMLUtils::nameMatches(current, "tasks") )
    {
        xmlNodePtr subNode = current->xmlChildrenNode;
        while(subNode != NULL)
        {
            if(XMLUtils::nameMatches(subNode, "task") )
            {
                AgentTask task = parseTask(doc, subNode);
                list.push_back(task);
            }
            subNode = subNode->next;
        }
    }
    return list;
}

void XMLReader::readAgentAttributes(xmlDocPtr doc, xmlNodePtr current)
{
    if( XMLUtils::nameMatches(current, "agent-attributes") )
    {
        xmlNodePtr subNode = current->xmlChildrenNode;
        while(subNode != NULL)
        {
            if(XMLUtils::nameMatches(subNode, "integer-attribute") )
            {
                AgentIntegerAttribute attribute = parseIntegerAttribute(doc, subNode);
                mAgentRoutingProblem.addIntegerAttribute(attribute);
            }

            subNode = subNode->next;
        }
    }
}

} // end namespace io
} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
