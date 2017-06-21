#ifndef TEMPL_SOLVERS_TEMPORAL_BOUNDS_HPP
#define TEMPL_SOLVERS_TEMPORAL_BOUNDS_HPP

#include <vector>
#include <string>

namespace templ {
namespace solvers {
namespace temporal {

/**
 * \class Bounds
 * \brief General structure to defining a lower and an upperBound
 */
class Bounds
{
    double mLowerBound;
    double mUpperBound;

public:
    typedef std::vector<Bounds> List;

    Bounds() {}

    Bounds(double lowerBound, double upperBound);

    void setLowerBound(double l) { mLowerBound = l; }
    void setUpperBound(double l) { mUpperBound = l; }

    double getLowerBound() const { return mLowerBound; }
    double getUpperBound() const { return mUpperBound; }

    bool operator==(const Bounds& other) const { return mLowerBound == other.mLowerBound &&
        mUpperBound == other.mUpperBound; }

    bool operator<(const Bounds& other) const;

    bool overlaps(const Bounds& other) const;

    bool isValid() const { return mLowerBound <= mUpperBound; }

    static bool equals(const List& a, const List& b) { return a == b; }
    static bool includesNegative(const List& a);

    /**
     * Reverse all intervals, i.e.
     * change [a,b] into [-b,-a] for each interval of a set v
     */
    static std::vector<Bounds> reverse(const List& v);


    std::string toString(size_t indent = 0) const;
    static std::string toString(const List& list, size_t indent = 0);
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_BOUNDS_HPP
