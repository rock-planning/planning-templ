#ifndef TEMPL_OBJECT_VARIABLES_LOCATION_HPP
#define TEMPL_OBJECT_VARIABLES_LOCATION_HPP

#include <templ/ObjectVariable.hpp>

namespace templ {
namespace object_variables {

class Location : public ObjectVariable
{
public:
    Location(const std::string& name, const std::string& subtype = "")
        : ObjectVariable(name, ObjectVariable::TypeTxt[LOCATION])
    {}

    virtual ~Location() {}
};

} // end namespace object_variables
} // end namespace templ
#endif // TEMPL_OBJECT_VARIABLES_LOCATION_HPP

