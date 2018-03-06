#include "SetNGL.hpp"

using namespace Gecode;
using namespace Gecode::Set;

namespace templ {
namespace solvers {
namespace csp {

ExcNGL::ExcNGL(Space& home, SetView x, int n)
      : ViewValNGL<SetView,int,PC_SET_ANY>(home,x,n) {}

ExcNGL::ExcNGL(Space& home, ExcNGL& ngl)
      : ViewValNGL<SetView,int,PC_SET_ANY>(home,ngl) {}


  NGL*
  ExcNGL::copy(Space& home) {
    return new (home) ExcNGL(home,*this);
  }
  NGL::Status
  ExcNGL::status(const Space&) const {
    // Is n not in the lub(x)?
    if (x.notContains(n))
      return NGL::SUBSUMED;
    else
      // Is n in the lub(x)?
      return x.contains(n) ? NGL::FAILED : NGL::NONE;
  }
  ExecStatus
  ExcNGL::prune(Space& home) {
    return me_failed(x.include(home,n)) ? ES_FAILED : ES_OK;
  }

}}}
