#include "IntervalConstraint.hpp"
#include <sstream>
#include <graph_analysis/EdgeTypeManager.hpp>

using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace temporal {


const graph_analysis::EdgeRegistration<IntervalConstraint> IntervalConstraint::msRegistration;

IntervalConstraint::IntervalConstraint()
    : Edge()
{}

IntervalConstraint::IntervalConstraint(const point_algebra::TimePoint::Ptr& source,
        const point_algebra::TimePoint::Ptr& target)
    : Edge(source, target)
{
}

std::string IntervalConstraint::toString() const
{
    return toString(0);
}

std::string IntervalConstraint::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "Interval Constraint: from " << getSourceVertex()->toString() << " to " << getTargetVertex()->toString() << std::endl;
    ss << hspace << "    " << serializeBounds();
        return ss.str();
}


bool IntervalConstraint::checkInterval(const Bounds& x)
{
    std::vector<Bounds>::const_iterator it = mIntervals.begin();
    for(; it != mIntervals.end(); ++it)
    {
        if (it->getLowerBound() == x.getLowerBound() && it->getUpperBound() == x.getUpperBound())
                {
                    return true;
                }
    }
    return false;
}

void IntervalConstraint::registerAttributes(graph_analysis::EdgeTypeManager* eManager) const
{
    eManager->registerAttribute(getClassName(), "bounds",
                (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&IntervalConstraint::serializeBounds,
                   (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&IntervalConstraint::deserializeBounds,
                   (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&IntervalConstraint::serializeBounds);

}

std::string IntervalConstraint::serializeBounds() const
{
    size_t numberOfIntervals = getIntervalsNumber();
    std::stringstream ss;
    ss << numberOfIntervals;
    for(size_t i = 0; i < numberOfIntervals; ++i)
    {
        ss << "[";
        ss << mIntervals[i].getLowerBound();
        ss << ",";
        ss << mIntervals[i].getUpperBound();
        ss << "]";
    }
    return ss.str();
}


void IntervalConstraint::deserializeBounds(const std::string& blob)
{
    size_t numberOfIntervals;
    std::stringstream ss(blob);
    ss >> numberOfIntervals;
    double lowerBound, upperBound;
    std::string filler;
    for(size_t i = 0; i < numberOfIntervals; ++i)
    {
        ss >> filler;
        ss >> lowerBound;
        ss >> filler;
        ss >> upperBound;
        ss >> filler;
        addInterval( Bounds(lowerBound, upperBound) );
    }
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
