#ifndef TEMPL_UTILS_CARTOGRAPHIC_MAPPING_HPP
#define TEMPL_UTILS_CARTOGRAPHIC_MAPPING_HPP

#include <map>
#include <base/Point.hpp>

namespace templ {
namespace utils {

class CartographicMapping
{
public:
    static const int RADIUS_MOON_IN_M; 

    enum Type { UNKNOWN, EARTH, MOON };
    static std::map<Type, std::string> TypeTxt;

    CartographicMapping(const std::string& radiusTypeName);

    CartographicMapping(Type type);

    base::Point latitudeLongitudeToMetric(const base::Point& point) const;

    static Type getTypeByName(const std::string& name);

private:
    Type mType;
};

} // end namespace utils
} // end namespace templ
#endif // TEMPL_UTILS_CARTOGRAPHIC_MAPPING_HPP
