#ifndef TEMPL_AGENT_ROUTING_ARP_IO_HPP
#define TEMPL_AGENT_ROUTING_ARP_IO_HPP

#include <vector>
#include <map>
#include <string>
#include "AgentRoutingProblem.hpp"
#include <templ/SharedPtr.hpp>

namespace templ {
namespace agent_routing {

namespace representation {
    enum Type { UNKNOWN = 0, XML };

    typedef std::string Suffix;
    extern std::map<Type, std::string> TypeTxt;

} // end namespace representation

class Writer
{
public:
    typedef shared_ptr<Writer> Ptr;

    virtual ~Writer();

    virtual void write(const std::string& path, const AgentRoutingProblem& arp) = 0;
};

class Reader
{
public:
    typedef shared_ptr<Reader> Ptr;
    virtual ~Reader();

    virtual void read(const std::string& path, AgentRoutingProblem& arp) = 0;
};

class ArpIO
{
public:
    typedef std::map<representation::Type, Writer::Ptr> WriterMap;
    typedef std::map<representation::Type, Reader::Ptr> ReaderMap;
    typedef std::map<representation::Suffix, representation::Type> SuffixMap;

    static void write(const std::string& path, const AgentRoutingProblem& arp, representation::Type representation = representation::XML);

    static void read(const std::string& path, AgentRoutingProblem& arp, representation::Type representation = representation::XML);

    static representation::Type getTypeFromSuffix(representation::Suffix suffix);
    static representation::Suffix getSuffix(representation::Type format);
    static std::string appendSuffix(const std::string& filename, representation::Type representation);
    static representation::Type getTypeFromFilename(const std::string& filename);

private:
    static WriterMap msWriters;
    static ReaderMap msReaders;

    static SuffixMap msSuffixes;
};

} // end namespace agent_routing
} // end namespaec templ
#endif // TEMPL_AGENT_ROUTING_ARP_IO_HPP
