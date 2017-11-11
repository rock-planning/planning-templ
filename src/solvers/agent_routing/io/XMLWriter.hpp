#ifndef TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP
#define TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP

#include "../ArpIO.hpp"

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

};

} // end namespace io
} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_IO_XML_WRITER_HPP
