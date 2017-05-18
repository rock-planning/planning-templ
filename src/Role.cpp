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
    ss << "Role: " << mName << " (";
    if(mModel != owlapi::model::IRI())
    {
        ss << mModel.getFragment();
    } else {
        ss << mModel;
    }
    ss << ")";
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

std::string Role::toString(const Role::Set& roles)
{
    std::stringstream ss;
    Role::Set::const_iterator cit = roles.begin();
    ss << "Roles: " << std::endl;
    for(; cit != roles.end(); ++cit)
    {
        ss << "    - " << cit->toString() << std::endl;
    }
    return ss.str();
}

organization_model::ModelPool Role::getModelPool(const Role::List& roles)
{
    organization_model::ModelPool modelPool;
    for(const Role& role : roles)
    {
        owlapi::model::IRI model = role.getModel();
        modelPool[model] +=1;
    }
    return modelPool;
}

bool Role::operator<(const Role& other) const
{
    if(mModel == other.mModel)
    {
        return mName < other.mName;
    }
    return mModel < other.mModel;
}

} // end namespace templ
