#include "Location.hpp"
#include <sstream>

namespace templ {
namespace symbols {
namespace constants {

Location::Location()
    : Constant("unknown", Constant::LOCATION)
    , mPosition( base::Point(base::Point::Zero()) )
{
}

Location::Location(const std::string& name, const base::Point& position)
    : Constant(name, Constant::LOCATION)
    , mPosition(position)
{
}

std::string Location::toString() const
{
    std::stringstream ss;
    ss << getInstanceName() << "(" << mPosition.x() << "," << mPosition.y() << ")";
    return ss.str();
}

} // end namespace constants
} // end namespace symbols
} // end namespace templ
