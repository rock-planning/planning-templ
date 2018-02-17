#ifndef TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP
#define TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP

#include <gecode/set.hh>
#include <gecode/search.hh>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>

namespace templ {
namespace solvers {
namespace csp {

/**
 * Allow to define a temporal constraint network based on a qualitative temporal
 * constraint network
 *
 * This CSP based variant allows solving a qualitative temporal constraint
 * network by simple <,=,> constraints on integer variables
 * i.e., each timepoint will be represented as IntVar, though the actual value
 * is only important to encode the relative relationship
 */
class TemporalConstraintNetwork : public Gecode::Space
{
public:
    /**
     * Construct CSP from existing qualitative temporal constraint network
     */
    TemporalConstraintNetwork(const temporal::QualitativeTemporalConstraintNetwork& tcn);

    /**
     * Copy constructor for Gecode
     */
    TemporalConstraintNetwork(bool shared, TemporalConstraintNetwork& other);

    /**
     * Deconstructor
     */
    virtual ~TemporalConstraintNetwork();

    /**
     * Check if the temporal constraint network is consistent, by searching for
     * 1 valid solution
     */
    static bool isConsistent(const temporal::QualitativeTemporalConstraintNetwork& tcn);

    /**
     * Copy call for Gecode
     */
    virtual Gecode::Space* copy(bool share);

    /**
     * Find a valid assignment for this temporal constraint network
     */
    TemporalConstraintNetwork* nextSolution();

    /**
     * Get the sorted list of timepoint of a qualitative temporal constraint
     * network
     */
    static std::vector<temporal::point_algebra::TimePoint::Ptr> getSortedList(const temporal::QualitativeTemporalConstraintNetwork& tcn);

    /**
     * Allow to partially sort a list of timepoints based on a valid assignment
     * of the qualitative temporal constraint network
     * Requires to solve the tcn (providing a valid solution);
     */
    static void sort(const temporal::QualitativeTemporalConstraintNetwork& tcn, std::vector<temporal::point_algebra::TimePoint::Ptr>& timepoints);

    /**
     * Allow to partially sort a list of timepoints based on a solved (!)
     * TemporalConstraintNetwork
     */
    static void sort(const TemporalConstraintNetwork* tcnSolution, std::vector<temporal::point_algebra::TimePoint::Ptr>& timepoints);

    /**
     * Get the value corresponding to a particular vertex/timepoint when a
     * solution has been found
     * \throw
     */
    uint32_t getValue(const graph_analysis::Vertex::Ptr& v) const;


private:
    /// List of vertices forming the qualitative temporal network
    std::vector<graph_analysis::Vertex::Ptr> mVertices;
    /// IntVarArray which represents the list of timepoints forming the
    /// qualitative temporal network
    Gecode::IntVarArray mTimepoints;

    /**
     * Get the index of the vertex in the locally cached vertex list
     * \return index if found
     * \throw std::runtime_error when vertex list is empty
     * \throw std::invalid_argument when vertex could not be found in the vertex
     * list
     */
    size_t getVertexIdx(const graph_analysis::Vertex::Ptr& ptr) const;

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP
