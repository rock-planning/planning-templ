#ifndef TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TEMPORAL_CONSTRAINT_NETWORK
#define TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TEMPORAL_CONSTRAINT_NETWORK

#include <templ/solvers/temporal/TemporalConstraintNetwork.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>

namespace templ {
namespace solvers {
namespace temporal {

/**
 * \class QualitativeTemporalConstraintNetwork
 * \brief A constraint network of qualitative timepoints
 * \details Automated Planning p 292
 * "A PA [Point Algebra] network (X,C) is consistent iff there is a set of
 * primitives p_ij \in r_ij, for each pair(i,j) such that every triple of
 * primitives verifies p_ij \in (p_ik o p_kj)
 */
class QualitativeTemporalConstraintNetwork : public TemporalConstraintNetwork
{
public:
    typedef boost::shared_ptr<QualitativeTemporalConstraintNetwork> Ptr;

    /**
     * Default constructor
     */
    QualitativeTemporalConstraintNetwork();

    /**
     * Get the known and consolidated qualitative constraint between two timepoints
     * \return consolidated timepoint constraint
     */
    point_algebra::QualitativeTimePointConstraint::Type getQualitativeConstraint(const point_algebra::TimePoint::Ptr& t1, const point_algebra::TimePoint::Ptr& t2);

    /**
     * Add timepoint constraint to the constraint network
     * \return Added constraint
     */
    point_algebra::QualitativeTimePointConstraint::Ptr addQualitativeConstraint(const point_algebra::TimePoint::Ptr& t1, const point_algebra::TimePoint::Ptr& t2, point_algebra::QualitativeTimePointConstraint::Type constraint);

    /**
     * Remove a qualitative constraint
     * Clears internal caches
     */
    void removeQualitativeConstraint(const point_algebra::QualitativeTimePointConstraint::Ptr& constraint);

    /** Check 3-path consistency withing the constraint graph
     * \return true if graph is consistent, false otherwise
     */
    bool isConsistent();

    /**
     * Check the consistency of a triangle of vertices
     */
    bool isConsistent(const std::vector<graph_analysis::Vertex::Ptr>& triangle);

    /** Check consistency between two vertices
     * \return true if constraints between two vertices are consistent, false otherwise
     */
    bool isConsistent(const graph_analysis::Vertex::Ptr& v0, const graph_analysis::Vertex::Ptr& v1);

    /**
     * Get the consistent constraint type between two vertices, i.e. takes
     * all edges into account that start at v0 and end at v1
     * \return composition constraint
     * \throw if the constraints between the two vertices are not consistent
     */
    point_algebra::QualitativeTimePointConstraint::Type getDirectionalConstraintType(const graph_analysis::Vertex::Ptr& v0, const graph_analysis::Vertex::Ptr& v1) const;

    /**
     * Get the consistent constraint between two vertices accouting for all
     * edges between these two vertices
     * \return composition constraint
     * \throw if the constraints between the two vertices are not consistent
     */
    point_algebra::QualitativeTimePointConstraint::Type getBidirectionalConstraintType(const graph_analysis::Vertex::Ptr& v0, const graph_analysis::Vertex::Ptr& v1) const;

private:
    typedef std::pair<graph_analysis::Vertex::Ptr, graph_analysis::Vertex::Ptr> VertexPair;
    typedef std::map< VertexPair, point_algebra::QualitativeTimePointConstraint::Type> ConstraintCache;

    /**
     * Check if consistency between a set of edges (multiedge) is given
     * for qualitative time point constraints
     * \return True if multiedge is consistent, false if not
     */
    bool isConsistent(const std::vector<graph_analysis::Edge::Ptr>& edges);

    point_algebra::QualitativeTimePointConstraint::Type updateConstraintCache(const graph_analysis::Vertex::Ptr& v1, const graph_analysis::Vertex::Ptr& v2, point_algebra::QualitativeTimePointConstraint::Type constraint);

    bool constraintUpdated(const VertexPair& pair) const { return mUpdatedConstraints.end() != std::find(mUpdatedConstraints.begin(), mUpdatedConstraints.end(), pair); }

    ConstraintCache mCompositionConstraints;
    std::vector<VertexPair> mCurrentUpdatedConstraints;
    std::vector<VertexPair> mUpdatedConstraints;
    bool mConsistencyChecked;

    uint32_t mTriangleCheckCount;
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TEMPORAL_CONSTRAINT_NETWORK
