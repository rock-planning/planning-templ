#include "ObjectVariable.hpp"
#include <boost/assign/list_of.hpp>

namespace templ {
namespace symbols {

std::map<ObjectVariable::Type, std::string> ObjectVariable::TypeTxt = boost::assign::map_list_of
    (UNKNOWN, "UNKNOWN")
    (LOCATION_CARDINALITY, "LocationCardinality")
    (LOCATION_NUMERIC_ATTRIBUTE, "LocationNumericAttribute")
    ;

ObjectVariable::ObjectVariable(const std::string& name, Type type)
    : Symbol(name, TypeTxt[type], Symbol::OBJECT_VARIABLE)
    , mObjectVariableType(type)
{}

ObjectVariable::Ptr ObjectVariable::getInstance(const std::string& name, Type type)
{
    return ObjectVariable::Ptr( new ObjectVariable(name, type));
}

bool ObjectVariable::equals(const Symbol::Ptr& symbol) const
{
    if(symbol->getType() == Symbol::OBJECT_VARIABLE)
    {
        ObjectVariable::Ptr other = dynamic_pointer_cast<ObjectVariable>(symbol);
        return getInstanceName() == other->getInstanceName() &&
            getType() == other->getType();
    }
    return false;
}

} // end namespace symbols
} // end namespace templ
