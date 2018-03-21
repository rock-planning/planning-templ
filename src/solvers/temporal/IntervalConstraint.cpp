#include "IntervalConstraint.hpp"
#include <sstream>
#include <graph_analysis/EdgeTypeManager.hpp>
#include <base-logging/Logging.hpp>

using namespace graph_analysis;

namespace templ {
namespace solvers {
namespace temporal {


const graph_analysis::EdgeRegistration<IntervalConstraint> IntervalConstraint::msRegistration;

IntervalConstraint::IntervalConstraint()
    : constraints::SimpleConstraint(Constraint::TEMPORAL_QUANTITATIVE)
{}

IntervalConstraint::IntervalConstraint(const point_algebra::TimePoint::Ptr& source,
        const point_algebra::TimePoint::Ptr& target)
    : constraints::SimpleConstraint(Constraint::TEMPORAL_QUANTITATIVE, source, target)
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
    ss << hspace << "Interval Constraint: from '" << getSourceVertex()->toString() << "' to '" << getTargetVertex()->toString() << "'" <<  std::endl;
    ss << hspace << "    " << serializeBounds();
        return ss.str();
}

void IntervalConstraint::addInterval(const Bounds& newInterval)
{
    // check if interval already exists
    if( mIntervals.end() == std::find(mIntervals.begin(), mIntervals.end(), newInterval))
    {
        mIntervals.push_back(newInterval);
    }
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
    std::stringstream ss;
    ss << mIntervals.size();
    for(const Bounds& bound : mIntervals)
    {
        ss << "[";
        ss << bound.getLowerBound();
        ss << ",";
        ss << bound.getUpperBound();
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
    char filler;
    for(size_t i = 0; i < numberOfIntervals; ++i)
    {
        ss >> filler;
        ss >> lowerBound;
        ss >> filler;
        ss >> upperBound;
        addInterval( Bounds(lowerBound, upperBound) );
    }
}

double IntervalConstraint::getLowerBound() const
{
    double minimum = std::numeric_limits<double>::max();
    for(const Bounds& bounds : mIntervals)
    {
        minimum = std::min(minimum, bounds.getLowerBound());
    }
    return minimum;
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
