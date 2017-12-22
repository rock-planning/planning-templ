#ifndef TEMPL_ROLE_HPP
#define TEMPL_ROLE_HPP

#include <owlapi/model/IRI.hpp>
#include <organization_model/ModelPool.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/set.hpp>

namespace templ {

/**
 * Role within the mission, representing an individual
 * system and the corresponding model
 */
class Role
{
    friend class boost::serialization::access;
    /// type of model this role fulfills
    owlapi::model::IRI mModel;
    /// id or the role
    size_t mId;

    std::string mName;
public:
    typedef std::vector<Role> List;
    typedef std::set<Role> Set;

    /**
     * Default constructor to allow usage in lists and maps
     */
    Role();

    /**
     * Preferred role constructor
     * \param name Name/Id of the role
     * \param model Model identification
     */
    Role(size_t id, const owlapi::model::IRI& model);

    const owlapi::model::IRI& getModel() const { return mModel; }
    size_t getId() const  { return mId; }
    const std::string& getName() const { return mName; }

    std::string toString() const;
    static std::string toString(const List& roles, size_t indent = 0);
    static std::string toString(const Set& roles, size_t indent = 0);
    static organization_model::ModelPool getModelPool(const List& roles);
    static organization_model::ModelPool getModelPool(const Set& roles);

    bool operator<(const Role& other) const;
    bool operator==(const Role& other) const { return mModel == other.mModel && mId == other.mId; }
    bool operator!=(const Role& other) const { return ! (*this == other); }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mModel;
        ar & mId;
        ar & mName;
    }

    /**
     * Creates the list of roles, that can be created from a given model pool,
     * assuming that it contains only types as keys
     */
    static List createRoles(const organization_model::ModelPool& modelPool);
};

typedef Role::Set Coalition;
typedef std::vector<Role::Set> CoalitionStructure;

} // end namespace templ
#endif // TEMPL_ROLE_HPP
