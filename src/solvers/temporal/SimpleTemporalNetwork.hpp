#ifndef TEREP_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP
#define TEREP_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP

#include <terep/solvers/ConstraintNetwork.hpp>
#include <terep/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>

namespace terep {
namespace solvers {
namespace temporal {

class SimpleTemporalNetwork : public ConstraintNetwork
{
public:

    point_algebra::TimePoint::Ptr createTimepoint(uint64_t lowerBound, uint64_t upperBound);

    void addQualitativeConstraint(point_algebra::TimePoint::Ptr t1, point_algebra::TimePoint::Ptr t2, point_algebra::QualitativeConstraintType constraint);

    bool isConsistent() const;
};

} // end namespace temporal
} // end namespace solvers
} // end namespace terep
#endif // TEREP_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP
