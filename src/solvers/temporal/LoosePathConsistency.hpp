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
    // compute the intersection between two sets of intervals
    static std::vector<Bounds> intersection(const std::vector<Bounds>& a, const std::vector<Bounds>& b);

    // check if an interval x is included in a set of intervals a
    static bool checkInterval(const std::vector<Bounds>& a, const Bounds& x);

    // compute the composition between two sets of intervals
    static std::vector<Bounds> composition(const std::vector<Bounds>& a, const std::vector<Bounds>& b);

    // compute the loose intersection between two sets of intervals
    static std::vector<Bounds> looseIntersection(const std::vector<Bounds>& a, const std::vector<Bounds>& b);

    // change [a,b] into [-b,-a] for each interval of a set v
    static std::vector<Bounds> reverseIntervals(const std::vector<Bounds>& v);

    // step 1 
    // computes the intersection over the composition of the interval constraints
    static TemporalConstraintNetwork intersectionNetwork(TemporalConstraintNetwork t);

    // step 2
    // compute loose intersection between two temp networks
    static TemporalConstraintNetwork looseNetwork(TemporalConstraintNetwork a, TemporalConstraintNetwork b);

    // checks if a temporal constraint network is consistent 
    // i.e. check if there is a T(i,j) (edge) which has no interval contraints assigned
    // returns true if tcn is consistent and otherwise false
    static bool checkConsistency(TemporalConstraintNetwork n);

    // brief implementation of loose path consistency algorithm
    // receives a temp constraint network and returns a tighter temp constraint network 
    static TemporalConstraintNetwork loosePathConsistency(TemporalConstraintNetwork t);

};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_LOOSE_PATH_CONSISTENCY
