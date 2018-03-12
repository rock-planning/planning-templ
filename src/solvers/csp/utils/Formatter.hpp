#pragma once

#include <gecode/minimodel.hh>
#include <gecode/set.hh>

#include "../../../Variable.hpp"
#include "../../../Symbol.hpp"
#include "../../../Role.hpp"
#include "../../FluentTimeResource.hpp"

namespace templ {
namespace solvers {
namespace csp {
namespace utils {

class Formatter
{
public:
    /**
     * Print a set of array in matrix form
     */
    static std::string toString(const std::vector<Gecode::IntVarArray>& arrays, const std::vector<Symbol::Ptr>& fluents, const std::vector<Variable::Ptr>& variables, const std::vector<std::string>& arrayLabels);

    /**
     * Print an array in matrix form
     */
    static std::string toString(const Gecode::IntVarArray& array, const std::vector<Symbol::Ptr>& fluents, const std::vector<Variable::Ptr>& variables);

    static std::string toString(const Gecode::SetVarArray& array, size_t columnSize);

    static std::string toString(const Gecode::ViewArray<Gecode::Set::SetView>& array, size_t columnSize);

    static std::string toString(const std::vector<Gecode::SetVarArray>& array, size_t columnSize);

    /**
     * Print an array of roles with respect to requirements
     */
    static std::string toString(const Gecode::IntVarArray& array,
            const Role::List& roles,
            const FluentTimeResource::List& requirements);

};

} // end namespace utils
} // end namespace csp
} // end namespace solvers
} // end namespace templ
