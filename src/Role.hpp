#ifndef TEMPL_ROLE_HPP
#define TEMPL_ROLE_HPP

#include <owlapi/model/IRI.hpp>
#include <organization_model/AtomicAgent.hpp>

namespace templ {

typedef organization_model::AtomicAgent Role;

typedef Role::Set Coalition;
typedef std::vector<Role::Set> CoalitionStructure;

} // end namespace templ
#endif // TEMPL_ROLE_HPP
