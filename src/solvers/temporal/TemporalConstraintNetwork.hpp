#ifndef TEMPL_SOLVERS_TEMPORAL_TEMPORAL_CONSTRAINT_NETWORK
#define TEMPL_SOLVERS_TEMPORAL_TEMPORAL_CONSTRAINT_NETWORK

#include <templ/solvers/ConstraintNetwork.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>

namespace templ {
namespace solvers {
namespace temporal {

class TemporalConstraintNetwork : public ConstraintNetwork
{
protected:
    graph_analysis::BaseGraph::Ptr mpDistanceGraph;

public:
    typedef boost::shared_ptr<TemporalConstraintNetwork> Ptr;

    TemporalConstraintNetwork();

    virtual ~TemporalConstraintNetwork();

    void addTimePoint(point_algebra::TimePoint::Ptr t);

    virtual bool isConsistent() { throw std::runtime_error("templ::solvers::temporal::TemporalConstraintNetwork::isConsistent is not implemented"); }
};

} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_TEMPORAL_CONSTRAINT_NETWORK
