#ifndef TEMPL_IO_FLUENT_TYPES_HPP
#define TEMPL_IO_FLUENT_TYPES_HPP

#include <cstdio>
#include <cstdint>
#include <string>

namespace templ {
namespace io {

/**
 * A 3D Location
 */
struct Location
{
    std::string id;
    int32_t x;
    int32_t y;
    int32_t z;

    std::string toString(uint32_t indent = 0) const;
};

} // end namespace io
} // end namespace templ
#endif // TEMPL_IO_FLUENT_TYPES_HPP
