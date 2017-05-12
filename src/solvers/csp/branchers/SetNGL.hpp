#ifndef TEMPL_SOLVERS_CSP_SET_NGL_HPP
#define TEMPL_SOLVERS_CSP_SET_NGL_HPP

#include <gecode/set.hh>

namespace templ {
namespace solvers {
namespace csp {

/// No-good literal for exclusion
    class ExcNGL : public Gecode::ViewValNGL<Gecode::Set::SetView,int,Gecode::Set::PC_SET_ANY> {
    public:
      /// Constructor for creation
      ExcNGL(Gecode::Space& home, Gecode::Set::SetView x, int n);
      /// Constructor for cloning \a ngl
      ExcNGL(Gecode::Space& home, bool share, ExcNGL& ngl);
      /// Test the status of the no-good literal
      virtual NGL::Status status(const Gecode::Space& home) const;
      /// Propagate the negation of the no-good literal
      virtual Gecode::ExecStatus prune(Gecode::Space& home);
      /// Create copy
      virtual Gecode::NGL* copy(Gecode::Space& home, bool share);
    };

}}}
#endif // TEMPL_SOLVERS_CSP_SET_NGL_HPP


