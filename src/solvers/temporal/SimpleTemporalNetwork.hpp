#ifndef TEMPL_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP
#define TEMPL_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP

#include "Bounds.hpp"
#include "TemporalConstraintNetwork.hpp"

namespace templ {
namespace solvers {
namespace temporal {

/**
 * \class SimpleTemporalNetwork
 * \brief A SimpleTemporalNetwork (STN) represents a set of
 * quantitative timepoint constraints given by a set of intervals
 */
class SimpleTemporalNetwork : public TemporalConstraintNetwork
{
public:
    SimpleTemporalNetwork();

    virtual ~SimpleTemporalNetwork() {}

    bool isConsistent();

    void addInterval(point_algebra::TimePoint::Ptr source, point_algebra::TimePoint::Ptr target, const Bounds& bound);

    /** Propagate and check for consistency using FloydWarshall algorithm
     * \return the resulting distance graph
     */
    graph_analysis::BaseGraph::Ptr propagate();

protected:
    /**
     * Check if the constraint network has a negative cycle, i.e.
     * telling that is is inconsistent
     */
    bool hasNegativeCycle();

};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_SIMPLE_TEMPORAL_NETWORK_HPP
