#include "ArpIO.hpp"
#include <boost/regex.hpp>
#include "io/XMLWriter.hpp"
#include "io/XMLReader.hpp"
#include <base-logging/Logging.hpp>

namespace templ {
namespace solvers {
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


void ArpIO::write(const std::string& filename, const AgentRoutingProblem& arp, representation::Type representation)
{
    WriterMap::const_iterator cit = msWriters.find(representation);
    if(cit != msWriters.end())
    {
        std::string filenameWithSuffix = ArpIO::appendSuffix(filename, representation);

        Writer::Ptr writer = cit->second;
        writer->write(filenameWithSuffix, arp);
    } else {
        std::stringstream ss;
        ss << "ArpIO: writing representation ";
        ss << "' " << representation::TypeTxt[representation] << "'";
        ss << " is not supported";

        throw std::runtime_error(ss.str());
    }

}

void ArpIO::read(const std::string& filename, AgentRoutingProblem& arp, representation::Type representation)
{
    if(representation == representation::UNKNOWN)
    {
        representation = getTypeFromFilename(filename);
        if(representation == representation::UNKNOWN)
        {
            throw std::invalid_argument("templ::solvers::agent_routing::ArpIO::read: "
                " reading from file '" + filename + "' failed since filename has no meaningful suffix and no representation is given");
        }
    }

    ReaderMap::const_iterator cit = msReaders.find(representation);
    if(cit != msReaders.end())
    {
        Reader::Ptr reader = cit->second;
        reader->read(filename, arp);
    } else {
        std::stringstream ss;
        ss << "ArpIO: reading representation ";
        ss << "' " << representation::TypeTxt[representation] << "'";
        ss << " is not supported";

        throw std::runtime_error(ss.str());
    }

}


representation::Type ArpIO::getTypeFromSuffix(representation::Suffix suffix)
{
    SuffixMap::const_iterator cit = msSuffixes.find(suffix);
    if(cit != msSuffixes.end())
    {
        return cit->second;
    }
    throw std::invalid_argument("templ::solvers::agent_routing::ArpIO::getTypeFromSuffix: "
        " unknown suffix '" + suffix + "'");
}

representation::Suffix ArpIO::getSuffix(representation::Type format)
{
    SuffixMap::const_iterator cit = msSuffixes.begin();
    for(; cit != msSuffixes.end(); ++cit)
    {
        if(cit->second == format)
        {
            return cit->first;
        }
    }
    throw std::runtime_error("templ::solvers::agent_routing::ArpIO::getSuffix: "
            " could not retrieve suffix for format '" + representation::TypeTxt[format] + "'");
}


std::string ArpIO::appendSuffix(const std::string& filename, representation::Type format)
{
    representation::Type fileType = getTypeFromFilename(filename);
    if(fileType != representation::UNKNOWN && (format == representation::UNKNOWN || format == fileType))
    {
        // Filename already comes with suffix -- and no other format is
        // requested
        return filename;
    }

    if(fileType == representation::UNKNOWN && format != representation::UNKNOWN)
    {
        // ok append
        std::string filenameWithSuffix = filename + "." + getSuffix(format);
        return filenameWithSuffix;
    }

    throw std::invalid_argument("templ::solvers::agent_routing::ArpIO::appendSuffix: "
                " could not append suffix for format '" + representation::TypeTxt[format] + "' to '" + filename + "'");
}

representation::Type ArpIO::getTypeFromFilename(const std::string& filename)
{
    boost::regex expression(".*\\.([a-z]+$)");

    boost::cmatch what;
    if(boost::regex_match(filename.c_str(), what, expression))
    {
        std::string suffix(what[1].first, what[1].second);
        LOG_DEBUG_S << "Found suffix of filename '" << filename << "' : " << suffix;
        return getTypeFromSuffix(suffix);
    } else {
        LOG_DEBUG_S << "No suffix found in filename '" << filename << "'";
    }
    return representation::UNKNOWN;
}

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
