#include "Role.hpp"
#include <sstream>

namespace templ {

Role::Role()
    : mModel()
    , mId(0)
    , mName("unknown")
{}

Role::Role(size_t id, const owlapi::model::IRI& model)
    : mModel(model)
    , mId(id)
{
    std::stringstream ss;
    ss << mModel.getFragment() << "_" << mId;
    mName = ss.str();
}

std::string Role::toString() const
{
    std::stringstream ss;
    ss << "Role: " << getName() << " (";
    if(mModel != owlapi::model::IRI())
    {
        ss << mModel.getFragment();
    } else {
        ss << mModel;
    }
    ss << ")";
    return ss.str();
}

std::string Role::toString(const Role::List& roles, size_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    if(roles.empty())
    {
        ss << hspace << "Roles: empty " << std::endl;
        return ss.str();
    }

    Role::List::const_iterator cit = roles.begin();
    ss << hspace << "Roles: " << std::endl;
    for(; cit != roles.end(); ++cit)
    {
        ss << hspace << "    - " << cit->toString() << std::endl;
    }
    return ss.str();
}

std::string Role::toString(const Role::Set& roles, size_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    if(roles.empty())
    {
        ss << hspace << "Roles: empty " << std::endl;
        return ss.str();
    }

    ss << hspace << "Roles: " << std::endl;
    Role::Set::const_iterator cit = roles.begin();
    for(; cit != roles.end(); ++cit)
    {
        ss << hspace << "    - " << cit->toString() << std::endl;
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

organization_model::ModelPool Role::getModelPool(const Role::Set& roles)
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
        return mId < other.mId;
    }
    return mModel < other.mModel;
}

Role::List Role::createRoles(const organization_model::ModelPool& modelPool)
{
    List roles;
    organization_model::ModelPool::const_iterator cit = modelPool.cbegin();
    for(;cit != modelPool.cend(); ++cit)
    {
        const owlapi::model::IRI& model = cit->first;
        size_t count = cit->second;

        // Update roles
        for(size_t i = 0; i < count; ++i)
        {
            Role role(i, model);
            roles.push_back(role);
        }
    }
    return roles;
}

} // end namespace templ
