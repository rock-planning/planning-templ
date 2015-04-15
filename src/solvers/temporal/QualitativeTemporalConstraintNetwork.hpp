#ifndef TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TEMPORAL_CONSTRAINT_NETWORK
#define TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TEMPORAL_CONSTRAINT_NETWORK

#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>

namespace templ {
namespace solvers {
namespace temporal {

class QualitativeTemporalConstraintNetwork : public TemporalConstraintNetwork
{
    /**
     * Check if consistency between a set of edges (multiedge) is given
     * for qualitative time point constraints
     * \return True if multiedge is consistent, false if not
     */
    bool isConsistent(std::vector<graph_analysis::Edge::Ptr> edges);

    /** Check consistency between two vertices of the constraint graph
     * \return true if graph is consistent, false otherwise
     */
    bool isConsistent(graph_analysis::Vertex::Ptr vertex0, graph_analysis::Vertex::Ptr vertex1, graph_analysis::BaseGraph::Ptr graph);

public:
    typedef boost::shared_ptr<QualitativeTemporalConstraintNetwork> Ptr;

    QualitativeTemporalConstraintNetwork();

    std::vector<point_algebra::QualitativeTimePointConstraint::Ptr> getConstraints(point_algebra::TimePoint::Ptr t1, point_algebra::TimePoint::Ptr t2);

    void addConstraint(point_algebra::TimePoint::Ptr t1, point_algebra::TimePoint::Ptr t2, point_algebra::QualitativeTimePointConstraint::Type constraint);

    /**
     * Get the symmetric constraint for an edge (defined by a pair of vertices)
     * This allows to check for path consistency 
     * \return The corresponding symmetric constraint
     */
    static point_algebra::QualitativeTimePointConstraint::Type getSymmetricConstraint(graph_analysis::BaseGraph::Ptr graph, graph_analysis::Vertex::Ptr first, graph_analysis::Vertex::Ptr second);

    /** Check 3-path consistency withing the constraint graph
     * \return true if graph is consistent, false otherwise
     */
    bool isConsistent();
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TEMPORAL_CONSTRAINT_NETWORK
