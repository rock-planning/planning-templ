#include "Idx.hpp"

namespace templ {
namespace solvers {
namespace csp {
namespace propagators {

std::string Idx::toString() const
{
    std::stringstream ss;
    ss << "Idx: " << idx() << " tp: "<< isTimepointIdx();
    return ss.str();
}

Idx::Idx(Gecode::Space& home, Gecode::Propagator& p,
        Gecode::Council<Idx>& c, int i, bool isTimepointIdx, SetVarArrayView x)
    : Gecode::Advisor(home, p, c)
    , mInfo(i << 1)
    , mIsTimepointIdx(isTimepointIdx)
    , x(x)
{
    // Subscribe to view
    x.subscribe(home, *this);
}

Idx::Idx(Gecode::Space& home, Idx& other)
    : Gecode::Advisor(home, other)
    , mInfo(other.mInfo)
    , mIsTimepointIdx(other.mIsTimepointIdx)
{
    x.update(home, other.x);
}

void Idx::dispose(Gecode::Space& home, Gecode::Council<Idx>& c)
{
    x.cancel(home, *this);
    Advisor::dispose(home, c);
}




} // end namespace propagators
} // end namespace csp
} // end namespace solvers
} // end namespace templ

