#ifndef TEMPL_UTILS_CARTOGRAPHIC_MAPPING_HPP
#define TEMPL_UTILS_CARTOGRAPHIC_MAPPING_HPP

#include <map>
#include <base/Point.hpp>

namespace templ {
namespace utils {

/**
 * \class CartographicMapping
 * \brief  Allow to map cartesian coordinates to latitude/longitude coordinates
 * \details This mapping utility class relies on the cartopgraphic mapping library proj4
 * The configuration for the moon is:
 \verbatim
     +proj=merc +ellps=sphere +a=1737.1E03 +b=1737.1E03 +units=m
 \endverbatim
 * \see https://github.com/OSGeo/proj.4
 * \see https://trac.osgeo.org/proj/
 */
class CartographicMapping
{
public:
    // Static constant of the radius of the moon
    static const double RADIUS_MOON_IN_M;
    static const double RADIUS_EARTH_IN_M;

    /// Type of planet
    enum Type { UNKNOWN, EARTH, MOON };
    static std::map<Type, std::string> TypeTxt;

    /**
     * Constructor from name
     * \param radiusTypeName by name, i.e. one of EARTH or MOON
     */
    CartographicMapping(const std::string& radiusTypeName);

    /**
     * Constructor from given type
     */
    CartographicMapping(Type type);

    /**
     * Convert a latitude/longitude given coordinate to a metric coordinate
     * \return metric coordinate (units: m)
     */
    base::Point latitudeLongitudeToMetric(const base::Point& point) const;

    /**
     * Get the radius type by name
     * \return Type of radius
     */
    static Type getTypeByName(const std::string& name);

private:
    /// Cartographic mapping for which planet type
    Type mType;
};

} // end namespace utils
} // end namespace templ
#endif // TEMPL_UTILS_CARTOGRAPHIC_MAPPING_HPP
