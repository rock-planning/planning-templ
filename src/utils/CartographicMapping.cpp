#include "CartographicMapping.hpp"
#include <proj_api.h>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string.hpp>
#include <base-logging/Logging.hpp>

namespace templ {
namespace utils {

const double CartographicMapping::RADIUS_MOON_IN_M = 1737.1E03;
const double CartographicMapping::RADIUS_EARTH_IN_M = 6371E03;

std::map<CartographicMapping::Type, std::string> CartographicMapping::TypeTxt = boost::assign::map_list_of
    (UNKNOWN, "UNKNOWN")
    (EARTH, "EARTH")
    (MOON, "MOON")
    ;

CartographicMapping::CartographicMapping(const std::string& radiusTypeName)
{
    mType = getTypeByName(radiusTypeName);
}

CartographicMapping::CartographicMapping(Type type)
    : mType(type)
{
}

base::Point CartographicMapping::latitudeLongitudeToMetric(const base::Point& point) const
{
    projPJ pj_merc, pj_latlong;

    switch(mType)
    {
        case MOON:
        {
            {
                std::stringstream ss;
                ss << "+proj=merc +ellps=sphere";
                ss << " +a=" << RADIUS_MOON_IN_M;
                ss << " +b=" << RADIUS_MOON_IN_M;
                ss << " +units=m";
                ss << " +lat_ts=" << point.x();
                if (!(pj_merc = pj_init_plus(ss.str().c_str())) )
                {
                    throw std::runtime_error("templ::utils::CartographMapping::latitudeLongitudeToMetric: "
                            " could not init mercator projection for moon");
                }
            }
            {
                std::stringstream ss;
                ss << "+proj=latlong +ellps=sphere";
                ss << " +a=" << RADIUS_MOON_IN_M;
                ss << " +b=" << RADIUS_MOON_IN_M;
                ss << " +units=m";
                if (!(pj_latlong = pj_init_plus(ss.str().c_str())) )
                {
                    throw std::runtime_error("templ::utils::CartographMapping::latitudeLongitudeToMetric: "
                            " could not init latitude/longitude projection for moon");
                }
            }
            break;
        }
        case EARTH:
        case UNKNOWN:
        {
            if (!(pj_merc = pj_init_plus("+proj=merc +ellps=clrk66 +lat_ts=33")) )
            {
                throw std::runtime_error("templ::utils::CartographMapping::latitudeLongitudeToMetric: "
                        " could not init mercator projection");
            }
            if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=clrk66")) )
            {
                throw std::runtime_error("templ::utils::CartographMapping::latitudeLongitudeToMetric: "
                        " could not init latitude/longitude projection");
            }
        }
        break;
    }

    double x = point.x();
    double y = point.y();

    x *= DEG_TO_RAD;
    y *= DEG_TO_RAD;

    pj_transform(pj_latlong, pj_merc, 1, 1, &x, &y, NULL);
    return base::Point(x,y,0.0);
}

CartographicMapping::Type CartographicMapping::getTypeByName(const std::string& _name)
{
    std::string name = _name;
    boost::to_upper(name);

    std::map<CartographicMapping::Type, std::string>::const_iterator cit = CartographicMapping::TypeTxt.begin();
    for(; cit != CartographicMapping::TypeTxt.end(); ++cit)
    {
        if(cit->second == name)
        {
            return cit->first;
        }
    }

    throw std::invalid_argument("templ::utils::CartographMapping::getTypeByName: could not find radius type named: '" + _name + "'");
}


} // end namespace utils
} // end namespace templ
