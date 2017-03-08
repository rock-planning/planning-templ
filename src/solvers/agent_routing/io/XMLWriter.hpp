#ifndef TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP
#define TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP

#include "../ArpIO.hpp"

namespace templ {
namespace agent_routing {
namespace io {

class XMLWriter : public Writer
{
public:
    XMLWriter();
    virtual ~XMLWriter();
    virtual void write(const std::string& path, const AgentRoutingProblem& arp);
};

} // end namespace io
} // end namespace agent_routing
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP
