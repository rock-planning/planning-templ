#include "ObjectVariable.hpp"

namespace templ {

ObjectVariable::Ptr ObjectVariable::getInstance(const std::string& name, const std::string& typeName)
{
    return ObjectVariable::Ptr( new ObjectVariable(name, typeName));
}

} // end namespace templ
