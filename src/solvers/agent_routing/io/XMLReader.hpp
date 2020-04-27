#ifndef TEMPL_AGENT_ROUTING_IO_XML_READER_HPP
#define TEMPL_AGENT_ROUTING_IO_XML_READER_HPP

#include "../ArpIO.hpp"
#include "../../../utils/XMLTCNUtils.hpp"

namespace templ {
namespace solvers {
namespace agent_routing {
namespace io {

class XMLReader : public Reader
{
public:
    XMLReader();
    virtual ~XMLReader();
    void read(const std::string& path, AgentRoutingProblem& arp);

    void readAgentTypes(xmlDocPtr doc, xmlNodePtr current);
    AgentType parseAgentType(xmlDocPtr doc, xmlNodePtr current);
    AgentIntegerAttribute parseIntegerAttribute(xmlDocPtr doc, xmlNodePtr);

    void readAgents(xmlDocPtr doc, xmlNodePtr current);
    Agent parseAgent(xmlDocPtr doc, xmlNodePtr current);

    AgentTask parseTask(xmlDocPtr doc, xmlNodePtr current);
    AgentTask::List parseTasks(xmlDocPtr doc, xmlNodePtr current);

    void readAgentAttributes(xmlDocPtr doc, xmlNodePtr current);
private:
    AgentRoutingProblem mAgentRoutingProblem;
};

} // end namespace io
} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_IO_XML_READER_HPP
