#include "Location.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace templ {
namespace symbols {
namespace constants {

Location::PtrList Location::msLocations;

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
    ss << getInstanceName();
        //<< "(" << std::setprecision(2) << mPosition.x()
        //<< std::setprecision(2) << "," << mPosition.y() << ")";
    return ss.str();
}

std::string Location::toString(size_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << getInstanceName();
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


Location::Ptr Location::create(const Location& location)
{
    PtrList::iterator cit = std::find_if(msLocations.begin(), msLocations.end(),
            [location](const Location::Ptr& other)
            {
                return location.getInstanceName() == other->getInstanceName();
            });

    if(cit != msLocations.end())
    {
        return *cit;
    } else {
        Location::Ptr locationPtr(new Location(location));
        msLocations.push_back(locationPtr);
        return locationPtr;
    }
}

Location::Ptr Location::create(const std::string& name, const base::Point& position)
{
    Location l(name, position);
    return create(l);
}



} // end namespace constants
} // end namespace symbols
} // end namespace templ
