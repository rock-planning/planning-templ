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
    /// name / identifier or the role
    std::string mName;
    /// type of model this role fulfills
    owlapi::model::IRI mModel;
public:
    typedef std::vector<Role> List;

    /**
     * Default constructor to allow usage in lists and maps
     */
    Role();

    /**
     * Preferred role constructor
     * \param name Name/Id of the role
     * \param model Model identification
     */
    Role(const std::string& name, const owlapi::model::IRI& model);

    const std::string& getName() const { return mName; }
    const owlapi::model::IRI& getModel() const { return mModel; }

    std::string toString() const;
    static std::string toString(const std::vector<Role>& roles);

    bool operator<(const Role& other) const;
    bool operator==(const Role& other) const { return mName == other.mName && mModel == other.mModel; }
    bool operator!=(const Role& other) const { return ! (*this == other); }
};

} // end namespace templ
#endif // TEMPL_ROLE_HPP
