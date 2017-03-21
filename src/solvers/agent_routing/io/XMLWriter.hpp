#ifndef TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP
#define TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP

#include "../ArpIO.hpp"
#include <libxml/xmlwriter.h>

#define ARP_XML_RESULT_CHECK(x, msg) \
    if(x < 0) \
    {\
        throw std::runtime_error("templ::solver_agent_routing::io::XMLWriter:" \
            " xml operation failed: " #msg ); \
    };

namespace templ {
namespace solvers {
namespace agent_routing {
namespace io {

class XMLWriter : public Writer
{
public:
    XMLWriter(const std::string& encoding = "UTF-8");
    virtual ~XMLWriter();

    virtual void write(const std::string& path, const AgentRoutingProblem& arp);

private:
    std::string mEncoding;

    xmlChar* convertInput(const char* in, const char* encoding);

    void writeComment(xmlTextWriterPtr writer, const std::string& comment);
    void writeCDATA(xmlTextWriterPtr writer, const std::string& cdata);
    void writeString(xmlTextWriterPtr writer, const std::string& string);

    void startElement(xmlTextWriterPtr writer, const std::string& element);
    void endElement(xmlTextWriterPtr writer);

    template<typename T>
    void writeAttribute(xmlTextWriterPtr writer, const std::string& key, T value)
    {
        std::stringstream ss;
        ss << value;
        ARP_XML_RESULT_CHECK( xmlTextWriterWriteAttribute(writer, BAD_CAST key.c_str(), BAD_CAST ss.str().c_str()), writeAttribute );
    }

};

} // end namespace io
} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP
