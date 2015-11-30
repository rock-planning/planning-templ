#ifndef TEMPL_SOLVERS_CSP_TEMPORAL_CSP_HPP
#define TEMPL_SOLVERS_CSP_TEMPORAL_CSP_HPP

#include <vector>
#include <gecode/set.hh>
#include <gecode/search.hh>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>

namespace templ {
namespace solvers {
namespace csp {

class TemporalCSP : public Gecode::Space
{
public:
    typedef temporal::QualitativeTemporalConstraintNetwork::Ptr Solution;
    typedef std::vector<Solution> SolutionList;

    TemporalCSP(const temporal::QualitativeTemporalConstraintNetwork::Ptr& tcn);

    TemporalCSP(bool share, TemporalCSP& s);

    virtual Gecode::Space* copy(bool share);

    static SolutionList solve(const temporal::QualitativeTemporalConstraintNetwork::Ptr& tcn);

    TemporalCSP* nextSolution();

protected:
    Solution getSolution();

private:
    temporal::QualitativeTemporalConstraintNetwork::Ptr mTCN;
    Gecode::IntVarArray mTemporalRelations;

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TEMPORAL_CSP_HPP
