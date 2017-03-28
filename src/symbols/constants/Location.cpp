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

std::string Location::toString(const Location::List& l, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "Locations:" << std::endl;
    List::const_iterator cit = l.begin();
    for(; cit != l.end(); ++cit)
    {
        ss << hspace << "    " << cit->toString() << std::endl;
    }
    return ss.str();
}

std::string Location::toString(const Location::PtrList& l, size_t indent)
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    ss << hspace << "Locations:" << std::endl;
    PtrList::const_iterator cit = l.begin();
    for(; cit != l.end(); ++cit)
    {
        ss << hspace << "    " << (*cit)->toString() << std::endl;
    }
    return ss.str();
}

} // end namespace constants
} // end namespace symbols
} // end namespace templ
