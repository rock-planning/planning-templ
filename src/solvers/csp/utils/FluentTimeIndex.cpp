#include "FluentTimeIndex.hpp"

namespace templ {
namespace solvers {
namespace csp {
namespace utils {

FluentTimeIndex::FluentTimeIndex(size_t rowOrColIndex,
        size_t fluentIndex, size_t timeIndex,
        size_t numberOfFluents, size_t numberOfTimepoints)
    : mNumberOfFluents(numberOfFluents)
    , mNumberOfTimepoints(numberOfTimepoints)
    , mIndex(fluentIndex, timeIndex)
    , mRowOrColIndex(rowOrColIndex)
{
}

FluentTimeIndex FluentTimeIndex::fromRowOrCol(size_t rowOrCol,
        size_t numberOfFluents, size_t numberOfTimepoints)
{
    size_t fluentIndex = rowOrCol%numberOfFluents;
    size_t timeIndex = (rowOrCol - fluentIndex) / numberOfFluents;

    return FluentTimeIndex(rowOrCol, fluentIndex, timeIndex, numberOfFluents, numberOfTimepoints);
}

size_t FluentTimeIndex::toRowOrColumnIndex(size_t fluentIndex, size_t timeIndex,
        size_t numberOfFluents, size_t numberOfTimepoints)
{
    return timeIndex*numberOfFluents + fluentIndex;
}

size_t FluentTimeIndex::toRowOrColumnIndex(size_t fluentIndex, size_t timeIndex) const
{
    return toRowOrColumnIndex(fluentIndex, timeIndex, mNumberOfFluents, mNumberOfTimepoints);
}

size_t FluentTimeIndex::toArrayIndex(size_t row, size_t col,
        size_t numberOfFluents, size_t numberOfTimepoints)
{
    size_t rowLength = numberOfFluents*numberOfTimepoints;
    return row*rowLength + col;
}

std::pair<size_t, size_t> FluentTimeIndex::fromArrayIndex(size_t index,
        size_t numberOfFluents, size_t numberOfTimepoints)
{
    size_t rowLength = numberOfFluents*numberOfTimepoints;
    size_t row = index/rowLength;
    size_t col = index%rowLength;

    std::pair<size_t,size_t> coordinate;
    coordinate.first = row;
    coordinate.second = col;

    return coordinate;
}

} // end namespace utils
} // end namespace csp
} // end namespace solvers
} // end namespace templ
