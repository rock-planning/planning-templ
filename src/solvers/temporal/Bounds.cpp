#include "Bounds.hpp"
#include <sstream>

namespace templ {
namespace solvers {
namespace temporal {

Bounds::Bounds(double lowerBound, double upperBound)
    : mLowerBound(lowerBound)
    , mUpperBound(upperBound)
{
}

std::string Bounds::toString(size_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << "[" << getLowerBound();
    ss << "," << getUpperBound() << "]";
    return ss.str();
}

std::string Bounds::toString(const List& list, size_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "Bounds:" << std::endl;
    for(const Bounds& bounds : list)
    {
        ss << bounds.toString(indent + 4) << std::endl;;
    }
    return ss.str();
}

bool Bounds::operator<(const Bounds& other) const
{
    if(getLowerBound() < other.getLowerBound())
    {
        return true;
    } else if(getLowerBound() == other.getLowerBound())
    {
        return getUpperBound() < other.getUpperBound();
    } else {
        return false;
    }
}

bool Bounds::overlaps(const Bounds& other) const
{
    if(mLowerBound == other.mLowerBound || mUpperBound == other.mUpperBound)
        return true;

    return !(mUpperBound <= other.mLowerBound || other.mUpperBound <= mLowerBound);
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ

