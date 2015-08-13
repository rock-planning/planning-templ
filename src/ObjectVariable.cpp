#include "ObjectVariable.hpp"
#include <boost/assign/list_of.hpp>

namespace templ {

std::map<ObjectVariable::Type, std::string> ObjectVariable::TypeTxt = boost::assign::map_list_of
    (UNKNOWN, "UNKNOWN")
    (LOCATION, "Location")
    (LOCATION_CARDINALITY, "LocationCardinality")
    ;

ObjectVariable::Ptr ObjectVariable::getInstance(const std::string& name, const std::string& typeName)
{
    return ObjectVariable::Ptr( new ObjectVariable(name, typeName));
}

} // end namespace templ
