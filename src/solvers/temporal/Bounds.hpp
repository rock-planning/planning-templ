#ifndef TEMPL_SOLVERS_TEMPORAL_BOUNDS_HPP
#define TEMPL_SOLVERS_TEMPORAL_BOUNDS_HPP

namespace templ {
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
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_BOUNDS_HPP
