#include "Session.hpp"

namespace templ {
namespace solvers {

Session::Session()
{}

Session::Session(const Mission::Ptr& mission)
    : mpMission(mission)
{}

} // end namespace solvers
} // end namespace templ
