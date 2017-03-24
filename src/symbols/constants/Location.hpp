#ifndef TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP
#define TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP

#include <templ/symbols/Constant.hpp>
#include <base/Point.hpp>
#include <vector>

namespace templ {
namespace symbols {
namespace constants {

class Location : public Constant
{
    base::Point mPosition;

public:
    typedef shared_ptr<Location> Ptr;
    typedef std::vector<Location> List;
    typedef std::vector<Ptr> PtrList;

    Location();

    Location(const std::string& name, const base::Point& position = base::Point::Zero());

    virtual ~Location() {}

    void setPosition(const base::Point& position) { mPosition = position; }
    const base::Point& getPosition() const { return mPosition; }

    virtual std::string toString() const;
};


} // end namespace constants
} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP

