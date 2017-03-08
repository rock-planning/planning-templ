#include "Location.hpp"

namespace templ {
namespace symbols {
namespace constants {

Location::Location()
    : Constant("unknown", Constant::LOCATION)
    , mPosition( base::Point() )
{}

Location::Location(const std::string& name, const base::Point& position)
    : Constant(name, Constant::LOCATION)
    , mPosition(position)
{
}

} // end namespace constants
} // end namespace symbols
} // end namespace templ
