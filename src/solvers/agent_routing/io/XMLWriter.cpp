#include "XMLWriter.hpp"
#include <libxml/encoding.h>
#include <base-logging/Logging.hpp>
#include <sstream>


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
    startElement(writer, "agent-routing-problem");
    writeComment(writer, "The general description of an agent vehicle routing problem");

    startElement(writer, "agent-attributes");
    writeComment(writer, "The commonly available attributes on the set of agents");
    {
        const std::vector<AgentIntegerAttribute>& attributes = arp.getAgentIntegerAttributes();
        std::vector<AgentIntegerAttribute>::const_iterator ait = attributes.begin();
        for(; ait != attributes.end(); ++ait)
        {
            const AgentIntegerAttribute& attribute = *ait;
            startElement(writer, "integer-attribute");
            writeAttribute(writer, "id", attribute.getId());
            writeAttribute(writer, "label", attribute.getLabel());
            writeAttribute(writer, "min", attribute.getMinValue());
            writeAttribute(writer, "max", attribute.getMaxValue());
            endElement(writer);
        }
    }
    endElement(writer);

    startElement(writer, "agent-types");
    const std::vector<AgentType>& types = arp.getAgentTypes();
    std::vector<AgentType>::const_iterator typesIt = types.begin();
    for(; typesIt != types.end(); ++typesIt)
    {
        const AgentType& type = *typesIt;
        const std::vector<AgentIntegerAttribute>& typeAttributes = type.getIntegerAttributes();
        std::vector<AgentIntegerAttribute>::const_iterator ait = typeAttributes.begin();
        startElement(writer, "agent-type");
        writeAttribute(writer, "id", type.getTypeId());

        for(; ait != typeAttributes.end(); ++ait)
        {
            const AgentIntegerAttribute& attribute = *ait;
            startElement(writer, "attribute");
            writeAttribute(writer, "id", attribute.getId());
            std::stringstream ss;
            ss << attribute.getValue();
            writeString(writer, ss.str());
            endElement(writer);
        }
        endElement(writer);
    }

    writeComment(writer, "Available agent types");
    //xmlTextWriterWriteFormatElement(writer, BAD_CAST "X_ORDER_ID", "%010d", 53535);
    endElement(writer);

    startElement(writer, "agents");
    writeComment(writer, "This is the description of all agent instance requirements");
    {
        const std::vector<Agent>& requirements = arp.getAgents();
        std::vector<Agent>::const_iterator cit = requirements.begin();
        for(; cit != requirements.end(); ++cit)
        {
            startElement(writer, "agent");
            writeAttribute(writer, "id", cit->getAgentId());
            writeAttribute(writer, "type", cit->getAgentType());

            startElement(writer, "tasks");
            const std::vector<AgentTask>& tasks = cit->getAgentTasks();
            std::vector<AgentTask>::const_iterator tit = tasks.begin();
            for(; tit != tasks.end(); ++tit)
            {
                const AgentTask& task = *tit;
                startElement(writer, "task");
                writeAttribute(writer, "priority", task.getTaskPriority());
                writeAttribute(writer, "duration", task.getTaskDuration());
                {
                    startElement(writer, "location");
                    writeAttribute(writer, "x", task.getLocation().getPosition().x());
                    writeAttribute(writer, "y", task.getLocation().getPosition().y());
                    writeString(writer, task.getLocation().getInstanceName());
                    endElement(writer);
                }
                {
                    startElement(writer, "arrival");
                    std::stringstream ss;
                    ss << task.getArrival()->getLabel();
                    writeString(writer, ss.str());
                    endElement(writer);
                }
                {
                    startElement(writer, "departure");
                    std::stringstream ss;
                    ss << task.getDeparture()->getLabel();
                    writeString(writer, ss.str());
                    endElement(writer);
                }


                endElement(writer);
            }
            endElement(writer);

            endElement(writer);
        }
    }

    endElement(writer);

    endElement(writer); // end agent-types
    xmlTextWriterEndDocument(writer);

    xmlFreeTextWriter(writer);
    xmlSaveFileEnc(path.c_str(), doc, mEncoding.c_str());

    // the formatting stage
    std::string formattedFile = path + ".formatted";
    std::string command = "`which xmllint` --encode UTF-8 --format " + path + " > " + formattedFile;

    LOG_INFO("Trying to format using xmllint: '%s'", command.c_str());
    if( system(command.c_str()) == 0 )
    {
        command = "mv " + formattedFile + " " + path;
        if( system(command.c_str()) )
        {
            throw std::runtime_error("templ::solvers::agent_routing::io::XMLWriter::write Failed to rename file after performing xmlling");
        }
    } else {
        LOG_INFO("ARP XML file '%s' written, but proper formatting failed -- make sure that xmllint is installed", path.c_str());
    }


    xmlFreeDoc(doc);
}


void XMLWriter::writeComment(xmlTextWriterPtr writer, const std::string& comment)
{
    xmlChar* xmlComment = convertInput(comment.c_str(), mEncoding.c_str());
    int rc = xmlTextWriterWriteComment(writer, xmlComment);
    if(rc < 0)
    {
        throw std::runtime_error("templ::agent_routing::io::XMLWriter::write: failed to"
                " write comment: '" + comment + "'");
    }
    if(xmlComment != NULL)
    {
        xmlFree(xmlComment);
    }
}

void XMLWriter::writeCDATA(xmlTextWriterPtr writer, const std::string& cdata)
{
    ARP_XML_RESULT_CHECK( xmlTextWriterWriteCDATA(writer, convertInput( cdata.c_str(), mEncoding.c_str() ) ) , writeCDATA);
}

void XMLWriter::writeString(xmlTextWriterPtr writer, const std::string& cdata)
{
    ARP_XML_RESULT_CHECK( xmlTextWriterWriteString(writer, convertInput( cdata.c_str(), mEncoding.c_str() ) ), writeString );
}

void XMLWriter::startElement(xmlTextWriterPtr writer, const std::string& element)
{
    int rc = xmlTextWriterStartElement(writer, BAD_CAST element.c_str());
    if (rc < 0)
    {
        throw std::runtime_error("templ::solvers::agent_routing::io::XMLWriter::startElement failed"
                " for element '" + element + "'");
    }
}

void XMLWriter::endElement(xmlTextWriterPtr writer)
{
    int rc = xmlTextWriterEndElement(writer);
    if (rc < 0)
    {
        throw std::runtime_error("templ::solvers::agent_routing::io::XMLWriter::endElement failed");
    }
}


xmlChar* XMLWriter::convertInput(const char *in, const char *encoding)
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
        throw std::runtime_error("templ::agent_routing::io::XMLWriter: no encoding handler found for" + std::string(encoding));
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

} // end namespace io
} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
