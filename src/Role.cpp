#include "Role.hpp"
#include <sstream>

namespace templ {

Role::Role()
    : mName("unknown")
    , mModel()
{}

Role::Role(const std::string& name, const owlapi::model::IRI& model)
    : mName(name)
    , mModel(model)
{}

std::string Role::toString() const
{
    std::stringstream ss;
    ss << "Role: " << mName << " (" << mModel.getFragment() << ")";
    return ss.str();
}

std::string Role::toString(const Role::List& roles)
{
    std::stringstream ss;
    Role::List::const_iterator cit = roles.begin();
    ss << "Roles: " << std::endl;
    for(; cit != roles.end(); ++cit)
    {
        ss << "    - " << cit->toString() << std::endl;
    }
    return ss.str();
}

} // end namespace templ
