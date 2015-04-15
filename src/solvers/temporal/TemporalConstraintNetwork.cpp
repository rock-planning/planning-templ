#include "TemporalConstraintNetwork.hpp"

namespace templ {
namespace solvers {
namespace temporal {

TemporalConstraintNetwork::TemporalConstraintNetwork()
    : mpDistanceGraph( new graph_analysis::lemon::DirectedGraph() )
{
}

TemporalConstraintNetwork::~TemporalConstraintNetwork()
{
}

void TemporalConstraintNetwork::addTimePoint(point_algebra::TimePoint::Ptr t)
{
    mpDistanceGraph->addVertex(t);
}

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
