#ifndef TEMPL_SOLVERS_CSP_PROPAGATORS_IDX
#define TEMPL_SOLVERS_CSP_PROPAGATORS_IDX

#include <gecode/int.hh>
#include <gecode/int/rel.hh>
#include <gecode/set.hh>
#include <set>

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

typedef Gecode::ViewArray<Gecode::Set::SetView> SetVarArrayView;

class Idx : public Gecode::Advisor
{
protected:
    // Info store the index and the boolean mark in the first bit,
    // thus doing bitshifting to retrieve index
    int mInfo;
    bool mIsTimepointIdx;

public:
    Idx(Gecode::Space& home,
            Gecode::Propagator& p,
            Gecode::Council<Idx>& c,
            int i,
            bool isTimepointIdx,
            SetVarArrayView x);

    Idx(Gecode::Space& home, Idx& a);
    bool isTimepointIdx() const { return mIsTimepointIdx; }
    bool isLocationIdx() const { return !mIsTimepointIdx; }

    bool isMarked() { return (mInfo & 1) != 0 ; }
    void mark(void) { mInfo |= 1 ; }
    void unmark(void) { mInfo &= ~1; }
    int idx(void) const { return mInfo >> 1; }

    void dispose(Gecode::Space& home, Gecode::Council<Idx>& c);

    std::string toString() const;

    SetVarArrayView x;
};

} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_PROPAGATORS_IDX
