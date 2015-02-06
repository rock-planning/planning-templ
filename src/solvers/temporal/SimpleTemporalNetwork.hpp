#ifndef TEMPL_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP
#define TEMPL_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP

#include <templ/solvers/ConstraintNetwork.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>
#include <templ/solvers/temporal/Bounds.hpp>

namespace templ {
namespace solvers {
namespace temporal {

class SimpleTemporalNetwork : public ConstraintNetwork
{
    graph_analysis::BaseGraph::Ptr mpDistanceGraph;

public:
    SimpleTemporalNetwork();

    ~SimpleTemporalNetwork() {}

    void addTimePoint(point_algebra::TimePoint::Ptr t);

    void addInterval(point_algebra::TimePoint::Ptr source, point_algebra::TimePoint::Ptr target, const Bounds& bound);

    void addQualitativeConstraint(point_algebra::TimePoint::Ptr t1, point_algebra::TimePoint::Ptr t2, point_algebra::QualitativeConstraintType constraint);

    static point_algebra::QualitativeConstraintType getSymmetricConstraint(graph_analysis::BaseGraph::Ptr graph, graph_analysis::Vertex::Ptr first, graph_analysis::Vertex::Ptr second);

    bool isConsistent();

    //
    // \return the resulting distance graph
    graph_analysis::BaseGraph::Ptr propagate();
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP
