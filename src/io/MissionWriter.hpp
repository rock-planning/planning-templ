#ifndef TEMPL_MISSION_WRITER_HPP
#define TEMPL_MISSION_WRITER_HPP

#include "../Mission.hpp"
#include <libxml/xmlwriter.h>

namespace templ {
namespace io {

class MissionWriter
{
public:
    static void write(const std::string& path, const Mission& mission, const std::string& encoding = "UTF-8");

};

} // end namespace io
} // end namespace templ
#endif // TEMPL_MISSION_WRITER_HPP
