#ifndef TEMPL_ROLE_HPP
#define TEMPL_ROLE_HPP

#include <owlapi/model/IRI.hpp>
#include <moreorg/AtomicAgent.hpp>

namespace templ {

typedef moreorg::AtomicAgent Role;

typedef Role::Set Coalition;
typedef std::vector<Role::Set> CoalitionStructure;

} // end namespace templ
#endif // TEMPL_ROLE_HPP
