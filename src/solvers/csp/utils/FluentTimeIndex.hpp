#pragma once

#include <cstdio>
#include <vector>
#include <stdint.h>

namespace templ {
namespace solvers {
namespace csp {
namespace utils {

class FluentTimeIndex
{
    size_t mNumberOfFluents;
    size_t mNumberOfTimepoints;

    // Local index of fluent and time
    std::pair<size_t,size_t> mIndex;

    // Index of the row or column
    size_t mRowOrColIndex;

    // Index in a linary array
    size_t mLinearArrayIndex;

public:
    FluentTimeIndex();

    FluentTimeIndex(size_t rowOrColumnIndex,
            size_t fluentIndex, size_t timeIndex,
            size_t numberOfFluents, size_t numberOfTimepoints);

    static FluentTimeIndex fromRowOrCol(size_t index,
            size_t numberOfFluents, size_t numberOfTimepoints );

    static size_t toRowOrColumnIndex(size_t fluentIndex, size_t timeIndex,
            size_t numberOfFluents, size_t numberOfTimepoints);

    size_t toRowOrColumnIndex(size_t fluentIndex, size_t timeIndex) const;

    size_t getFluentIndex() { return mIndex.first; }
    size_t getTimeIndex() { return mIndex.second; }

    static size_t toArrayIndex(size_t row, size_t col,
            size_t numberOfFluents, size_t numberOfTimepoints);

    std::pair<size_t,size_t> fromArrayIndex(size_t index,
            size_t numberOfFluents, size_t numberOfTimepoints);
};

} // end namespace utils
} // end namespace csp
} // end namespace solvers
} // end namespace templ
