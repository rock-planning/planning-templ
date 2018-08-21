#include "TimePointComparator.hpp"
#include <base-logging/Logging.hpp>
#include <algorithm>
#include "../Interval.hpp"

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

TimePointComparator::TimePointComparator(const TemporalConstraintNetwork::Ptr& tcn)
    : mpTemporalConstraintNetwork(tcn)
{
    if(mpTemporalConstraintNetwork && !mpTemporalConstraintNetwork->isConsistent())
    {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePointComparator: given constraint network is not consistent -- cannot construct comparator");
    }
}

void TimePointComparator::sort(std::vector<point_algebra::TimePoint::Ptr>& timepoints) const
{
    std::sort(timepoints.begin(), timepoints.end(), [this](const point_algebra::TimePoint::Ptr& a, const point_algebra::TimePoint::Ptr& b)
            {
                if(a == b)
                {
                    return false;
                }
                return this->lessThan(a,b);
            }
    );
}

bool TimePointComparator::equals(const TimePoint::Ptr& t0, const TimePoint::Ptr& t1) const
{
    return t0->equals(t1);
}

bool TimePointComparator::greaterThan(const TimePoint::Ptr& t0, const TimePoint::Ptr& t1) const
{
    if(!t0)
    {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePointComparator::greaterThan: timepoint t0 is NULL");
    } else if(!t1)
    {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePointComparator::greaterThan: timepoint t1 is NULL");
    }

    if(t0->getType() != t1->getType())
    {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePointComparator::greaterThan: cannot compare different types of TimePoints");
    }

    // same timepoints
    if(equals(t0,t1))
    {
        return false;
    }

    if(t0->getType() == TimePoint::QUANTITATIVE)
    {
        return t0->getLowerBound() > t1->getUpperBound();
    } else if(t1->getType() == TimePoint::QUALITATIVE)
    {
        if(!mpTemporalConstraintNetwork)
        {
            throw std::runtime_error("templ::solvers::temporal::point_algebra::TimePointComparator::greaterThan: comparing qualitive timepoints, but not QualitativeTimePointConstraintNetwork given to comparator");
        }

        graph_analysis::Edge::PtrList existingEdges = mpTemporalConstraintNetwork->getGraph()->getEdges(t0,t1);
        QualitativeTimePointConstraint::Ptr constraint = mpTemporalConstraintNetwork->addQualitativeConstraint(t0, t1, point_algebra::QualitativeTimePointConstraint::Greater);
        bool consistent = mpTemporalConstraintNetwork->isConsistent();

        // Remove temporary addition
        graph_analysis::Edge::PtrList tempEdges = mpTemporalConstraintNetwork->getGraph()->getEdges(t0,t1);
        for(const graph_analysis::Edge::Ptr& e : tempEdges)
        {
            mpTemporalConstraintNetwork->getGraph()->removeEdge(e);
        }

        // Restore previous status
        for(const graph_analysis::Edge::Ptr& e : existingEdges)
        {
            mpTemporalConstraintNetwork->getGraph()->addEdge(e);
        }
        return consistent;
    } else {
        throw std::invalid_argument("templ::solvers::temporal::point_algebra::TimePointComparator::greaterThan: cannot compare this type of TimePoints");
    }
}

bool TimePointComparator::hasIntervalOverlap(const TimePoint::Ptr& a_start, const TimePoint::Ptr& a_end, const TimePoint::Ptr& b_start, const TimePoint::Ptr& b_end) const
{
//    if(TimePointComparator::inInterval(a_start, b_start, b_end))
//    {
//        return true;
//    } else if(TimePointComparator::inInterval(a_end, b_start, b_end))
//    {
//        return true;
//    } else if(TimePointComparator::inInterval(b_start, a_start, a_end))
//    {
//        return true;
//    } else if(TimePointComparator::inInterval(b_end, a_start, a_end))
//    {
//        return true;
//    }
//    return false;
    // assuming that a_start <= a_end and b_start <= b_end
    // a consistent db entails a_end <= b_start or b_end <= a_start
    // (Automated Planning p.331 Def 14.8)
    // a - b
    //return (TimePointComparator::lessThan(b_start, a_end) && TimePointComparator::lessThan(a_start, b_end)) || (TimePointComparator::lessThan(a_start, b_end) && TimePointComparator::lessThan(b_start, a_end));
    if(TimePointComparator::equals(a_start, b_start) || TimePointComparator::equals(a_end, b_end) || TimePointComparator::equals(a_end, b_start) || TimePointComparator::equals(a_start, b_end))
    {
        return true;
    }
    return !(TimePointComparator::lessOrEqual(a_end, b_start) || TimePointComparator::lessOrEqual(b_end, a_start));
}

Interval TimePointComparator::getIntervalOverlap(const TimePoint::Ptr& a_start, const TimePoint::Ptr& a_end, const TimePoint::Ptr& b_start, const TimePoint::Ptr& b_end) const
{
    Interval interval;
    if( TimePointComparator::equals(a_start, b_start)
        || TimePointComparator::lessThan(a_start, b_start))
    {
        interval.setFrom(b_start);
    } else {
        interval.setFrom(a_start);
    }

    if( TimePointComparator::equals(a_end, b_end)
            || TimePointComparator::lessThan(a_end, b_end))
    {
        interval.setTo(a_end);
    } else {
        interval.setTo(b_end);
    }
    interval.setTimePointComparator(*this);
    return interval;
}

bool TimePointComparator::inInterval(const TimePoint::Ptr& t0, const TimePoint::Ptr& i_start, const TimePoint::Ptr& i_end) const
{
    if(TimePointComparator::greaterOrEqual(t0, i_start) && TimePointComparator::lessOrEqual(t0, i_end))
    {
        return true;
    }
    return false;
}

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
