#include "Role.hpp"

namespace templ {

std::ostream& operator<<(std::ostream& os, const Role::Set& roles)
{
    os << "RoleSet [";
    size_t i = 0;
    for(const Role& role : roles)
    {
        os << role.toString();
        if(i < roles.size() - 1)
        {
            os << ", ";
        }
        ++i;
    }
    os << "]";
    return os;
}

} // end namespace templ
