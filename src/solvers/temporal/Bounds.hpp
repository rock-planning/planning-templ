#ifndef TEREP_SOLVERS_TEMPORAL_BOUNDS_HPP
#define TEREP_SOLVERS_TEMPORAL_BOUNDS_HPP

namespace terep {
namespace solvers {
namespace temporal {

class Bounds
{
    double mLowerBound;
    double mUpperBound;

public:
    Bounds(double lowerBound, double upperBound);

    double getLowerBound() const { return mLowerBound; } 
    double getUpperBound() const { return mUpperBound; }
};

} // end namespace temporal
} // end namespace solvers
} // end namespace terep
#endif // TEREP_SOLVERS_TEMPORAL_BOUNDS_HPP
