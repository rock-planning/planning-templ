#ifndef TEMPL_AGENT_ROUTING_IO_XML_READER_HPP
#define TEMPL_AGENT_ROUTING_IO_XML_READER_HPP

#include "../ArpIO.hpp"

namespace templ {
namespace agent_routing {
namespace io {

class XMLReader : public Reader
{
public:
    XMLReader();
    virtual ~XMLReader();
    void read(const std::string& path, AgentRoutingProblem& arp);

private:

};

} // end namespace io
} // end namespace agent_routing
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_IO_XML_READER_HPP
