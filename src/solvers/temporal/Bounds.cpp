#include "Bounds.hpp"
#include <sstream>
#include <limits>
#include <algorithm>

namespace templ {
namespace solvers {
namespace temporal {

Bounds::Bounds()
    : mLowerBound(std::numeric_limits<double>::min())
    , mUpperBound(std::numeric_limits<double>::max())
{}

Bounds::Bounds(double lowerBound, double upperBound)
    : mLowerBound(lowerBound)
    , mUpperBound(upperBound)
{
}

std::string Bounds::toString(size_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << "[" << mLowerBound;
    ss << "," << mUpperBound << "]";
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

bool Bounds::equals(const List& a, const List& b)
{
    return std::all_of(a.begin(), a.end(), [&b](List::value_type a)
            {
                return std::find(b.begin(), b.end(), a) != b.end();
            });
}

bool Bounds::includesNegative(const List& list)
{
    for(const Bounds& b : list)
    {
        if(b.mLowerBound < 0 || b.mUpperBound < 0)
        {
            return true;
        }
    }
    return false;
}

Bounds::List Bounds::reverse(const List& boundsList)
{
    std::vector<Bounds> result;
    for(const Bounds& b : boundsList)
    {
        result.push_back(Bounds(- b.getUpperBound(), -b.getLowerBound()));
    }
    return result;
}


} // end namespace temporal
} // end namespace solvers
} // end namespace templ

