#ifndef TEMPL_TIMEPOINT_COMPARATOR_HPP
#define TEMPL_TIMEPOINT_COMPARATOR_HPP

#include <map>
#include <templ/Timepoint.hpp>

namespace templ {

class TimepointComparator
{
    std::map<Timepoint::Label, Timepoint::LabelList> mAliases;

public:
    virtual bool equals(const Timepoint& t0, const Timepoint& t1) const;

    virtual bool greaterThan(const Timepoint& t0, const Timepoint& t1) const;

    bool greaterThanOrEqual(const Timepoint& t0, const Timepoint& t1) const { return equals(t0,t1) || greaterThan(t0, t1); }

    bool lessThanOrEqual(const Timepoint& t0, const Timepoint& t1) const { return greaterThanOrEqual(t1, t0); }

    bool lessThan(const Timepoint& t0, const Timepoint& t1) const { return greaterThan(t1,t0); }

    virtual bool hasIntervalOverlap(const Timepoint& a_start, const Timepoint& a_end, const Timepoint& b_start, const Timepoint& b_end) const;
};

} // end namespace templ
#endif // TEMPL_TIMEPOINT_COMPARATOR_HPP
