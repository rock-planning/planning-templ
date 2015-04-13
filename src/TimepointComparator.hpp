#ifndef TEMPL_TIMEPOINT_COMPARATOR_HPP
#define TEMPL_TIMEPOINT_COMPARATOR_HPP

namespace templ {

class TimepointComparator
{
public:
    virtual bool equals(const Timepoint& t0, const Timepoint& t1) const;

    virtual bool hasIntervalOverlap(const Timepoint& a_start, const Timepoint& a_end, const Timepoint& b_start, const Timepoint& b_end) const;
};

} // end namespace templ
#endif // TEMPL_TIMEPOINT_COMPARATOR_HPP
