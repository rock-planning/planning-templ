#include <templ/TimepointComparator.hpp>

namespace templ {

bool TimepointComparator::equals(const Timepoint& t0, const Timepoint& t1) const
{
    return false;
}

bool TimepointComparator::greaterThan(const Timepoint& t0, const Timepoint& t1) const
{
    return false;
}

bool TimepointComparator::hasIntervalOverlap(const Timepoint& a_start, const Timepoint& a_end, const Timepoint& b_start, const Timepoint& b_end) const
{

    return true;
}

} // end namespace templ
