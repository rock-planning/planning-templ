#include "ArpIO.hpp"
#include "io/XMLWriter.hpp"
#include "io/XMLReader.hpp"


namespace templ {
namespace agent_routing {

namespace representation {

std::map<Type, std::string> TypeTxt = {
    {UNKNOWN, "UNKNOWN"},
    {XML, "XML"}
};

} // end namespace representation

std::map<representation::Suffix, representation::Type> ArpIO::msSuffixes = {
      {"xml", representation::XML}
};

Writer::~Writer()
{}

Reader::~Reader()
{}

ArpIO::WriterMap ArpIO::msWriters = {
    {representation::XML, Writer::Ptr(new io::XMLWriter())}
};

ArpIO::ReaderMap ArpIO::msReaders = {
    {representation::XML, Reader::Ptr( new io::XMLReader())}
};


void ArpIO::write(const std::string& path, const AgentRoutingProblem& arp, representation::Type representation)
{
}

void ArpIO::read(const std::string& path, const AgentRoutingProblem& arp, representation::Type representation)
{
}

} // end namespace agent_routing
} // end namespace templ
