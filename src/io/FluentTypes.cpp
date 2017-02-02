#include "FluentTypes.hpp"
#include <sstream>

namespace templ {
namespace io {

std::string Location::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "Location: " << id;
    return ss.str();
}

} // end namespace io
} // end namespace templ

