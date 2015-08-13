#ifndef TEMPL_ROLE_HPP
#define TEMPL_ROLE_HPP

#include <owlapi/model/IRI.hpp>

namespace templ {

/**
 * Role within the mission, representing an individual
 * system and the corresponding model
 */
class Role
{
    std::string mName;
    owlapi::model::IRI mModel;
public:
    typedef std::vector<Role> List;

    Role();
    Role(const std::string& name, const owlapi::model::IRI& model);

    std::string toString() const;
    static std::string toString(const std::vector<Role>& roles);

    bool operator<(const Role& other) const { return mName < other.mName; }
};

} // end namespace templ
#endif // TEMPL_ROLE_HPP
