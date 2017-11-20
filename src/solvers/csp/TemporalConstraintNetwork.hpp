#ifndef TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP
#define TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP

#include <gecode/set.hh>
#include <gecode/search.hh>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>

namespace templ {
namespace solvers {
namespace csp {

class TemporalConstraintNetwork : public Gecode::Space
{
public:
    TemporalConstraintNetwork(const temporal::QualitativeTemporalConstraintNetwork& tcn);

    TemporalConstraintNetwork(bool shared, TemporalConstraintNetwork& other);

    static bool isConsistent(const temporal::QualitativeTemporalConstraintNetwork& tcn);

    virtual Gecode::Space* copy(bool share);

    TemporalConstraintNetwork* nextSolution();

    /**
     * Get the sorted list
     */
    static std::vector<temporal::point_algebra::TimePoint::Ptr> getSortedList(const temporal::QualitativeTemporalConstraintNetwork& tcn);

    static void sort(const temporal::QualitativeTemporalConstraintNetwork& tcn, std::vector<temporal::point_algebra::TimePoint::Ptr>& timepoints);

    uint32_t getValue(const graph_analysis::Vertex::Ptr& v);

private:
    std::vector<graph_analysis::Vertex::Ptr> mVertices;
    Gecode::IntVarArray mTimepoints;

    size_t getVertexIdx(const graph_analysis::Vertex::Ptr& ptr) const;

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_TEMPORAL_CONSTRAINT_NETWORK_HPP
