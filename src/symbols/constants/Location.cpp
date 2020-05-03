#include "Location.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include "../../utils/CartographicMapping.hpp"

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


Location::Ptr Location::create(const Location& l)
{
    Location::Ptr location = get(l.getInstanceName());
    if(location)
    {
        return location;
    } else {
        Location::Ptr locationPtr = make_shared<Location>(l);
        msLocations.push_back(locationPtr);
        return locationPtr;
    }
}

Location::Ptr Location::create(const std::string& name, const base::Point& position)
{
    Location l(name, position);
    return create(l);
}

Location::Ptr Location::create(const std::string& name,
        double latitude, double longitude, const std::string& radius)
{
    utils::CartographicMapping mapping(radius);
    base::Point point(latitude, longitude, 0.0);
    base::Point position = mapping.latitudeLongitudeToMetric(point);

    Location l(name, position);
    return create(l);
}

Location::Ptr Location::get(const std::string& name)
{
    if(name.empty())
    {
        return Location::Ptr();
    }

    PtrList::iterator cit = std::find_if(msLocations.begin(), msLocations.end(),
            [name](const Location::Ptr& other)
            {
                return name == other->getInstanceName();
            });

    if(cit != msLocations.end())
    {
        return *cit;
    } else {
        return Location::Ptr();
    }
}

} // end namespace constants
} // end namespace symbols
} // end namespace templ
