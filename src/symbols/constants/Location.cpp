#include "Location.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include "../../utils/CartographicMapping.hpp"

namespace templ {
namespace symbols {
namespace constants {

Location::PtrList Location::msLocations;

Location::Location()
    : Constant("unknown", Constant::LOCATION)
    , mPosition( base::Point(base::Point::Zero()) )
    , mCoordinateType(CARTESIAN)
    , mRadius(0.0)
{
}

Location::Location(const std::string& name, const base::Point& position)
    : Constant(name, Constant::LOCATION)
    , mPosition(position)
    , mCoordinateType(CARTESIAN)
    , mRadius(0.0)
{
}

Location::Location(const std::string& name, const base::Point& position, double radius)
    : Constant(name, Constant::LOCATION)
    , mPosition(position)
    , mCoordinateType(LATLONG)
    , mRadius(radius)
{
    if(mRadius == 0.0)
    {
        mRadius = utils::CartographicMapping::RADIUS_EARTH_IN_M;
    }

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
    double radius_in_m;
    if(radius == "earth")
    {
        radius_in_m = utils::CartographicMapping::RADIUS_EARTH_IN_M;
    } else if(radius == "moon")
    {
        radius_in_m = utils::CartographicMapping::RADIUS_MOON_IN_M;
    } else {
        throw std::invalid_argument("templ::symbols::constants::Location::create" \
                "radius '" + radius + "' is not known");
    }

    base::Point position(latitude, longitude, 0.0);

    Location l(name, position, radius_in_m);
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

double Location::getDistance(const Location& a, const Location& b)
{
    if(a.getCoordinateType() != b.getCoordinateType())
    {
        throw std::runtime_error("templ::symbols::constants::Location::getDistance" \
                " cannot compute distance for different coordinate types");
    }

    switch(a.getCoordinateType())
    {
        case LATLONG:
            return Location::getSphericalDistance(a,b);
        case CARTESIAN:
        default:
            return (a.getPosition() - b.getPosition()).norm();
    }

}

double Location::getSphericalDistance(const Location& a, const Location& b)
{
    if(a.getRadius() != b.getRadius())
    {
        throw std::runtime_error("templ::symbols::constants::Location::getSphericalDistance:" \
                "mismatch of radius between LATLONG coordinates");
    }

    if(a.getPosition() == b.getPosition())
    {
        return 0.0;
    }


    // According to the spherical cosine law
    double lat1 = a.getPosition().x();
    double lon1 = a.getPosition().y();

    double lat2 = b.getPosition().x();
    double lon2 = b.getPosition().y();

    double s_1 = (lat1+90.0) * M_PI/180.0;
    double s_2 = (lat2+90.0) * M_PI/180.0;

    double C = std::fabs(lon1-lon2)*M_PI/180.0;

    double cos12 = cos(s_1)*cos(s_2);
    double sin12 = sin(s_1)*sin(s_2)*cos(C);

    double dist = std::acos(cos12 + sin12);

    return dist*a.getRadius();
}

} // end namespace constants
} // end namespace symbols
} // end namespace templ
