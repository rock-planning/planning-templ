#include "CartographicMapping.hpp"

#include <proj.h>
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
    // https://proj.org/development/migration.html

    /* Create the context. */
    /* You may set C=PJ_DEFAULT_CTX if you are sure you will     */
    /* use PJ objects from only one thread                       */
    PJ_CONTEXT* ctx = proj_context_create();


    std::string pj_cart, pj_latlong;
    switch(mType)
    {
       // https://proj.org/operations/projections/fahey.html
        case MOON:
        {
            {
                std::stringstream ss;
                ss << "+proj=fahey +ellps=sphere";
                ss << " +a=" << RADIUS_MOON_IN_M;
                ss << " +b=" << RADIUS_MOON_IN_M;
                ss << " +units=m";
                ss << " +lat_ts=" << point.x();
               pj_cart = ss.str();
            }
            {
                std::stringstream ss;
                ss << "+proj=latlong +ellps=sphere";
                ss << " +a=" << RADIUS_MOON_IN_M;
                ss << " +b=" << RADIUS_MOON_IN_M;
                ss << " +units=m";
               pj_latlong = ss.str();
            }
            break;
        }
        case EARTH:
        case UNKNOWN:
        {
           pj_cart= "+proj=fahey +ellps=clrk66 +lat_ts=33";
            pj_latlong = "+proj=latlong +ellps=clrk66";
        }
        break;
    }

    PJ* G = proj_create_crs_to_crs(ctx, pj_latlong.c_str(), pj_cart.c_str(), NULL);

    double lon = point.x();
    double lat = point.y();

    /* Prepare the input */
    PJ_COORD c_in;
    c_in.lpzt.z = 0.0;
    c_in.lpzt.t = HUGE_VAL; // important only for time-dependent projections
    c_in.lp.lam = lon;
    c_in.lp.phi = lat;

    // Compute cartesian coordinates
    PJ_COORD c_out = proj_trans(G, PJ_FWD, c_in);

    LOG_DEBUG_S << "Input: longitude: " << lon << "(degrees)"
               << "latitude: " << lat << "(degrees)"
               << "Output: x: " << c_out.xy.x
               << "y: " << c_out.xy.y;
    proj_destroy(G);
    return base::Point(c_out.xy.x, c_out.xy.y,0.0);
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
