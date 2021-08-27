#ifndef TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP
#define TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP

#include "../Constant.hpp"
#include <base/Point.hpp>
#include <vector>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace templ {
namespace symbols {
namespace constants {

/**
 * \class Location
 * \brief A location constant
 */
class Location : public Constant
{
public:
    typedef shared_ptr<Location> Ptr;
    typedef std::vector<Location> List;
    typedef std::vector<Ptr> PtrList;

    enum CoordinateType { LATLONG, CARTESIAN };

    friend class boost::serialization::access;

    Location();

    Location(const std::string& name, const base::Point& position =
            base::Point::Zero());

    Location(const std::string& name,
            const base::Point& position,
            double radius);

    virtual ~Location() = default;

    void setPosition(const base::Point& position) { mPosition = position; }

    /**
     * Get the current associated position with this constant
     */
    const base::Point& getPosition() const { return mPosition; }

    CoordinateType getCoordinateType() const { return mCoordinateType; }

    virtual std::string toString() const;
    virtual std::string toString(size_t indent) const;

    static std::string toString(const List& l, size_t indent = 0);

    static std::string toString(const PtrList& l, size_t indent = 0);

    /**
      * Get a location by name
      */
    static Location::Ptr get(const std::string& name);

    static Location::Ptr create(const Location& location);

    static Location::Ptr create(const std::string& name,
            const base::Point& position = base::Point::Zero());

    static Location::Ptr create(const std::string& name,
            double latitude, double longitude, const std::string& radius = "earth");

    static double getDistance(const Location& a, const Location& b);

    /**
     * Get the radius for locations, that are defined in LATLONG
     */
    double getRadius() const { return mRadius; }

    static double getSphericalDistance(const Location& a, const Location& b);

    template<class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        if(version >= 1)
        {
            ar & mCoordinateType;
            ar & mRadius;
        }
        ar & mPosition.x();
        ar & mPosition.y();
        ar & mPosition.z();
        ar & first;
        ar & second;
    }

    template<class Archive>
    void load(Archive& ar, const unsigned int version)
    {
        if(version >= 1)
        {
            ar & mCoordinateType;
            ar & mRadius;
        }

        ar & mPosition.x();
        ar & mPosition.y();
        ar & mPosition.z();
        ar & first;
        ar & second;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

protected:
    static PtrList msLocations;

private:
    base::Point mPosition;
    CoordinateType mCoordinateType;
    double mRadius;

};

//BOOST_SERIALIZATION_SHARED_PTR(Location);
//BOOST_CLASS_VERSION(Location,1);


} // end namespace constants
} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP

