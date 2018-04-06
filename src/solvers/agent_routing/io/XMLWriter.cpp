#include "XMLWriter.hpp"
#include <libxml/encoding.h>
#include <base-logging/Logging.hpp>
#include <sstream>
#include <qxcfg/utils/XMLUtils.hpp>

using namespace qxcfg::utils;

namespace templ {
namespace solvers {
namespace agent_routing {
namespace io {

XMLWriter::XMLWriter(const std::string& encoding)
    : Writer()
    , mEncoding(encoding)
{}

XMLWriter::~XMLWriter()
{
}

void XMLWriter::write(const std::string& path, const AgentRoutingProblem& arp)
{
    xmlTextWriterPtr writer;
    xmlDocPtr doc;

    doc = xmlNewDoc(BAD_CAST XML_DEFAULT_VERSION);
    if(doc == NULL)
    {
        throw std::runtime_error("templ::agent_routing::io::XMLWriter::write: error creating"
                " xml document");
    }

    // Create a new XmlWriter for uri with no compression
    writer = xmlNewTextWriterDoc(&doc, 0);
    if(writer == NULL)
    {
        throw std::runtime_error("templ::agent_routing::io::XMLWriter::write: failed to open file '"
                + path + "'");
    }

    // Start the document with the xml default for the version and given encoding
    int rc = xmlTextWriterStartDocument(writer, NULL, mEncoding.c_str(), NULL);
    if(rc < 0)
    {
        throw std::runtime_error("templ::agent_routing::io::XMLWriter::write: failed to"
                " start document:  '" + path + "'");
    }
    XMLUtils::startElement(writer, "agent-routing-problem");
    XMLUtils::writeComment(writer, "The general description of an agent vehicle routing problem", mEncoding);

    XMLUtils::startElement(writer, "agent-attributes");
    XMLUtils::writeComment(writer, "The commonly available attributes on the set of agents", mEncoding);
    {
        const std::vector<AgentIntegerAttribute>& attributes = arp.getAgentIntegerAttributes();
        std::vector<AgentIntegerAttribute>::const_iterator ait = attributes.begin();
        for(; ait != attributes.end(); ++ait)
        {
            const AgentIntegerAttribute& attribute = *ait;
            XMLUtils::startElement(writer, "integer-attribute");
            XMLUtils::writeAttribute(writer, "id", attribute.getId());
            XMLUtils::writeAttribute(writer, "label", attribute.getLabel());
            XMLUtils::writeAttribute(writer, "min", attribute.getMinValue());
            XMLUtils::writeAttribute(writer, "max", attribute.getMaxValue());
            XMLUtils::endElement(writer);
        }
    }
    XMLUtils::endElement(writer);

    XMLUtils::startElement(writer, "agent-types");
    const std::vector<AgentType>& types = arp.getAgentTypes();
    std::vector<AgentType>::const_iterator typesIt = types.begin();
    for(; typesIt != types.end(); ++typesIt)
    {
        const AgentType& type = *typesIt;
        const std::vector<AgentIntegerAttribute>& typeAttributes = type.getIntegerAttributes();
        std::vector<AgentIntegerAttribute>::const_iterator ait = typeAttributes.begin();
        XMLUtils::startElement(writer, "agent-type");
        XMLUtils::writeAttribute(writer, "id", type.getTypeId());

        for(; ait != typeAttributes.end(); ++ait)
        {
            const AgentIntegerAttribute& attribute = *ait;
            XMLUtils::startElement(writer, "attribute");
            XMLUtils::writeAttribute(writer, "id", attribute.getId());
            std::stringstream ss;
            ss << attribute.getValue();
            XMLUtils::writeString(writer, ss.str(), mEncoding);
            XMLUtils::endElement(writer);
        }
        XMLUtils::endElement(writer);
    }

    XMLUtils::writeComment(writer, "Available agent types", mEncoding);
    //xmlTextWriterWriteFormatElement(writer, BAD_CAST "X_ORDER_ID", "%010d", 53535);
    XMLUtils::endElement(writer);

    XMLUtils::startElement(writer, "agents");
    XMLUtils::writeComment(writer, "This is the description of all agent instance requirements", mEncoding);
    {
        const std::vector<Agent>& requirements = arp.getAgents();
        std::vector<Agent>::const_iterator cit = requirements.begin();
        for(; cit != requirements.end(); ++cit)
        {
            XMLUtils::startElement(writer, "agent");
            XMLUtils::writeAttribute(writer, "id", cit->getAgentId());
            XMLUtils::writeAttribute(writer, "type", cit->getAgentType());

            XMLUtils::startElement(writer, "tasks");
            const std::vector<AgentTask>& tasks = cit->getAgentTasks();
            std::vector<AgentTask>::const_iterator tit = tasks.begin();
            for(; tit != tasks.end(); ++tit)
            {
                const AgentTask& task = *tit;
                XMLUtils::startElement(writer, "task");
                XMLUtils::writeAttribute(writer, "priority", task.getTaskPriority());
                XMLUtils::writeAttribute(writer, "duration", task.getTaskDuration());
                {
                    XMLUtils::startElement(writer, "location");
                    XMLUtils::writeAttribute(writer, "x", task.getLocation().getPosition().x());
                    XMLUtils::writeAttribute(writer, "y", task.getLocation().getPosition().y());
                    XMLUtils::writeString(writer, task.getLocation().getInstanceName(), mEncoding);
                    XMLUtils::endElement(writer);
                }
                {
                    XMLUtils::startElement(writer, "arrival");
                    std::stringstream ss;
                    ss << task.getArrival()->getLabel();
                    XMLUtils::writeString(writer, ss.str(), mEncoding);
                    XMLUtils::endElement(writer);
                }
                {
                    XMLUtils::startElement(writer, "departure");
                    std::stringstream ss;
                    ss << task.getDeparture()->getLabel();
                    XMLUtils::writeString(writer, ss.str(), mEncoding);
                    XMLUtils::endElement(writer);
                }


                XMLUtils::endElement(writer);
            }
            XMLUtils::endElement(writer);

            XMLUtils::endElement(writer);
        }
    }

    XMLUtils::endElement(writer);

    XMLUtils::endElement(writer); // end agent-types
    xmlTextWriterEndDocument(writer);

    xmlFreeTextWriter(writer);
    xmlSaveFileEnc(path.c_str(), doc, mEncoding.c_str());
    xmlFreeDoc(doc);

    XMLUtils::lint(path);
}


} // end namespace io
} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
