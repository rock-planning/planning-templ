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
    base::Point mPosition;

public:
    typedef shared_ptr<Location> Ptr;
    typedef std::vector<Location> List;
    typedef std::vector<Ptr> PtrList;

    friend class boost::serialization::access;

    Location();

    Location(const std::string& name, const base::Point& position = base::Point::Zero());

    virtual ~Location() {}

    void setPosition(const base::Point& position) { mPosition = position; }

    /**
     * Get the current associated position with this constant
     */
    const base::Point& getPosition() const { return mPosition; }

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

    template<class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        ar & mPosition.x();
        ar & mPosition.y();
        ar & mPosition.z();
        ar & first;
        ar & second;
    }

    template<class Archive>
    void load(Archive& ar, const unsigned int version)
    {
        ar & mPosition.x();
        ar & mPosition.y();
        ar & mPosition.z();
        ar & first;
        ar & second;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

protected:
    static PtrList msLocations;
};

//BOOST_SERIALIZATION_SHARED_PTR(Location);

} // end namespace constants
} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP

