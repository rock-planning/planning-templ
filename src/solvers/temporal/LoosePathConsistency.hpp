#ifndef TEMPL_SOLVERS_TEMPORAL_LOOSE_PATH_CONSISTENCY
#define TEMPL_SOLVERS_TEMPORAL_LOOSE_PATH_CONSISTENCY

#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>

namespace templ {
namespace solvers {
namespace temporal {

class LoosePathConsistency
{
/*  Loose Path Consistency Algorithm
 *  Input: A temporal constraint network N
 *  Output: A tighter temporal constraint network equivalent to N (stronger than ULT)
 *  Steps:
 *
 *  N2 <- N
 *  repeat
 *      N <- N2
 *      Compute N1 by assigning T1(i,j) = intersection after k between (composition(C(i,k),C(k,j)) for all i,j)
 *      Compute N2 by assigning T2(i,j) = looseIntersection(C(i,j),T1(i,j)) for all i,j
 *  until inconsistency or no interval removed
 */
public:
    /** compute the intersection between two sets of intervals
     * latex symbol: \oplus
     */
    static std::vector<Bounds> intersection(const std::vector<Bounds>& a, const std::vector<Bounds>& b);

    /**
     *  check if an interval x is included in a set of intervals a
     */
    static bool isIncluded(const std::vector<Bounds>& a, const Bounds& x);

    /**
     * Compute the composition between two sets of intervals
     *
     * latex symbol: \otimes
     */
    static std::vector<Bounds> composition(const std::vector<Bounds>& a, const std::vector<Bounds>& b);

    /**
     * Compute a fully disjoint set of intervals by merging overlapping ones
     * using the mininum of the lower bounds and maximum of the upper bounds
     */
    static std::vector<Bounds> computeDisjointIntervals(const Bounds::List& list);

    /**
     * Compute the loose intersection between two sets of intervals
     * (decribed by the triangle operator in the paper)
     *
     * here a -> I, b -> S
     *
     * looseIntersection: a_i intersect with b
     \f[
        \forall i : I^{'}_i=[L_i,U_i]\text{, where } [L_i,U_i] \text{are the lower and upper bounds of the intersection } I_i \oplus S
     \f]
     */
    static std::vector<Bounds> looseIntersection(const std::vector<Bounds>& a, const std::vector<Bounds>& b);

    /** step 1
     * computes the intersection over the composition of the interval constraints
     *
    \f[
        \forall i \le j : c^{'}_{i \rightarrow j} \leftarrow \bigcap_{\forall k} \left(c_{i \rightarrow k } \otimes c_{k \rightarrow j} \right)

    \f]
    */
    static TemporalConstraintNetwork intersectionNetwork(TemporalConstraintNetwork t);

    // step 2
    // compute loose intersection between two temp networks
    static TemporalConstraintNetwork looseIntersectionNetwork(const TemporalConstraintNetwork& a, const TemporalConstraintNetwork& b);

    // checks if a temporal constraint network is consistent
    // i.e. check if there is a T(i,j) (edge) which has no interval contraints assigned
    // returns true if tcn is consistent and otherwise false
    static bool checkConsistency(TemporalConstraintNetwork n);

    /**
     *  \brief implementation of loose path consistency algorithm
     *  use a temp constraint network and returns a tighter temp constraint network
     */
    static TemporalConstraintNetwork loosePathConsistency(const TemporalConstraintNetwork& t);

};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_LOOSE_PATH_CONSISTENCY
