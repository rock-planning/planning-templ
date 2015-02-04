#include "SimpleTemporalNetwork.hpp"

using namespace terep::solvers::temporal::point_algebra;

namespace terep {
namespace solvers {
namespace temporal {

TimePoint::Ptr SimpleTemporalNetwork::createTimepoint(uint64_t lowerBound, uint64_t upperBound)
{
    return TimePoint::Ptr( new TimePoint(lowerBound, upperBound) );
}

void SimpleTemporalNetwork::addQualitativeConstraint(TimePoint::Ptr t1, TimePoint::Ptr t2, QualitativeConstraintType constraintType)
{
    QualitativeTimePointConstraint::Ptr constraint;
    switch(constraintType)
    {
        case GreaterOrEqual:
            constraint = QualitativeTimePointConstraint::create(t2, t1, Less);
        case LessOrEqual:
            constraint = QualitativeTimePointConstraint::create(t2, t1, Greater);
        default:
            constraint = QualitativeTimePointConstraint::create(t1, t2, constraintType);
    }

    addConstraint(constraint);
}

bool SimpleTemporalNetwork::isConsistent() const
{
    //// Path consistency
    //while(true)
    //{
    //    graph_analysis::EdgeIterator::Ptr constraintIt = getConstraintIterator();
    //    while(constraintIt.next())
    //    {
    //        Edge::Ptr firstEdge = constraintIt->current();

    //        graph_analysis::EdgeIterator::Ptr innerConstraintIt = getConstraintIterator();
    //        while(innerConstraintIt.next())
    //        {
    //            Edge::Ptr secondEdge = innerConstraintIt->current();
    //            
    //            
    //        }
    //    }
    //}
    return true;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace terep
