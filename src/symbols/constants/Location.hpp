#ifndef TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP
#define TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP

#include <templ/symbols/Constant.hpp>
#include <base/Point.hpp>

namespace templ {
namespace symbols {
namespace constants {

class Location : public Constant
{
    base::Point mPosition;

public:
    typedef boost::shared_ptr<Location> Ptr;

    Location(const std::string& name, const base::Point& position = base::Point())
        : Constant(name, Constant::LOCATION)
        , mPosition(position)
    {}

    virtual ~Location() {}

    void setPosition(const base::Point& position) { mPosition = position; }
    const base::Point& getPosition() const { return mPosition; }
};

} // end namespace constants
} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_CONSTANTS_LOCATION_HPP

