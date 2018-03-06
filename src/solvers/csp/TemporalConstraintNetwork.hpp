#ifndef TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP
#define TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP

#include <gecode/set.hh>
#include <gecode/search.hh>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>

namespace templ {
namespace solvers {
namespace csp {

class TemporalConstraintNetwork;

/**
 * \class TemporalConstraintNetworkBase
 * \details An implementation of the core concepts that are later wrapped in a
 * Gecode::Space
 * However, to reuse the functionality in another Space, we opt to using this
 * base class implementation
 */
class TemporalConstraintNetworkBase
{
public:
    TemporalConstraintNetworkBase();

    TemporalConstraintNetworkBase(const temporal::QualitativeTemporalConstraintNetwork& tcn);

    /**
     * Construct CSP from existing qualitative temporal constraint network
     * and allow embedding into another space
     */
    TemporalConstraintNetworkBase(const temporal::QualitativeTemporalConstraintNetwork& tcn,
            Gecode::Space& space,
            Gecode::IntVarArray& timepoints);

    /**
     * Deconstructor
     */
    virtual ~TemporalConstraintNetworkBase();

    /**
     * Add constraints
     */
    void addConstraints(const temporal::QualitativeTemporalConstraintNetwork& tcn,
            Gecode::Space& space,
            Gecode::IntVarArray& timepoints);

    /**
     * Check if the temporal constraint network is consistent, by searching for
     * 1 valid solution
     */
    static bool isConsistent(const temporal::QualitativeTemporalConstraintNetwork& tcn);

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
    static void sort(const Gecode::IntVarArray& timepointArray,
            std::vector<temporal::point_algebra::TimePoint::Ptr>& timepoints,
            graph_analysis::Vertex::PtrList& verticesCache);

    /**
     * Get the value corresponding to a particular vertex/timepoint when a
     * solution has been found
     * \throw
     */
    static uint32_t getValue(const graph_analysis::Vertex::Ptr& v,
            const Gecode::IntVarArray& timepoints,
            graph_analysis::Vertex::PtrList& verticesCache
            );

    /**
     * Get the index of the vertex in the locally cached vertex list
     * \param ptr vertex for which the id should be retrieved
     * \param vertexCache Cache which shall be updated and queried
     * \return index if found
     * \throw std::runtime_error when vertex list is empty
     * \throw std::invalid_argument when vertex could not be found in the vertex
     * list
     */
    static size_t getVertexIdx(const graph_analysis::Vertex::Ptr& ptr,
            graph_analysis::Vertex::PtrList& verticesCache
            );

    /**
     * Set the vertices / cache of vertices, based on the input network
     */
    void setVertices(const graph_analysis::Vertex::PtrList& v) { mVertices = v; };

    /**
     * Get the current set of vertices / timepoints
     */
    const graph_analysis::Vertex::PtrList& getVertices() const { return mVertices; }

    /**
     * Translate a solution to a qualitative temporal constraint network
     */
    temporal::QualitativeTemporalConstraintNetwork::Ptr translate(const Gecode::IntVarArray& solution);

protected:
    /// List of vertices forming the qualitative temporal network
    mutable std::vector<graph_analysis::Vertex::Ptr> mVertices;

};

/**
 * Allow to define a temporal constraint network based on a qualitative temporal
 * constraint network
 *
 * This CSP based variant allows solving a qualitative temporal constraint
 * network by simple <,=,> constraints on integer variables
 * i.e., each timepoint will be represented as IntVar, though the actual value
 * is only important to encode the relative relationship
 */
class TemporalConstraintNetwork : public Gecode::Space, public TemporalConstraintNetworkBase
{
    friend class TemporalConstraintNetworkBase;

public:
    /**
     * Construct CSP from existing qualitative temporal constraint network
     */
    TemporalConstraintNetwork(const temporal::QualitativeTemporalConstraintNetwork& tcn);

    /**
     * Copy constructor for Gecode
     */
    TemporalConstraintNetwork(TemporalConstraintNetwork& other);

    /**
     * Deconstructor
     */
    virtual ~TemporalConstraintNetwork();

    /**
     * Copy call for Gecode
     */
    virtual Gecode::Space* copy(void);

    /**
     * Find a valid assignment for this temporal constraint network
     */
    TemporalConstraintNetwork* nextSolution();

    /**
     * Get timepoints
     * \return timepoints
     */
    Gecode::IntVarArray& getTimepoints() { return mTimepoints; }

    /**
     * Compute a temporal constraint network without timegaps from the existing solution
     */
    temporal::TemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() const;

protected:
    /// IntVarArray which represents the list of timepoints forming the
    /// qualitative temporal network
    Gecode::IntVarArray mTimepoints;
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP
