#include "TemporalConstraintNetwork.hpp"
#include "../../solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp"

using namespace graph_analysis;
using namespace templ::solvers::temporal;

namespace templ {
namespace solvers {
namespace csp {

TemporalConstraintNetwork::TemporalConstraintNetwork(const temporal::QualitativeTemporalConstraintNetwork& tcn)
    : Gecode::Space()
    , mVertices(tcn.getGraph()->getAllVertices())
    , mTimepoints(*this, mVertices.size(), 0, mVertices.size() - 1)
{
    EdgeIterator::Ptr edgeIt = tcn.getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {
        point_algebra::QualitativeTimePointConstraint::Ptr constraint = dynamic_pointer_cast<point_algebra::QualitativeTimePointConstraint>(edgeIt->current());
        if(!constraint)
        {
            throw std::runtime_error("templ::solvers::csp::TemporalConstraintNetwork: encountered an edge which is not a QualitativeTimePointConstraint");
        }

        size_t sourceIdx = getVertexIdx( constraint->getSourceVertex() );
        size_t targetIdx = getVertexIdx( constraint->getTargetVertex() );

        //std::cout << "Add constraint: " << point_algebra::QualitativeTimePointConstraint::TypeTxt[ constraint->getType() ] <<
        //   " from: " << sourceIdx << " to: " << targetIdx << std::endl;

        switch(constraint->getType())
        {
            case point_algebra::QualitativeTimePointConstraint::Empty:
                this->fail();
                break;
            case point_algebra::QualitativeTimePointConstraint::Less:
                Gecode::rel(*this, mTimepoints[sourceIdx], Gecode::IRT_LE, mTimepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::LessOrEqual:
                Gecode::rel(*this, mTimepoints[sourceIdx], Gecode::IRT_LQ, mTimepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::Greater:
                Gecode::rel(*this, mTimepoints[sourceIdx], Gecode::IRT_GR, mTimepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::GreaterOrEqual:
                Gecode::rel(*this, mTimepoints[sourceIdx], Gecode::IRT_GQ, mTimepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::Equal:
                Gecode::rel(*this, mTimepoints[sourceIdx], Gecode::IRT_EQ, mTimepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::Distinct:
                Gecode::rel(*this, mTimepoints[sourceIdx], Gecode::IRT_NQ, mTimepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::Universal:
                break;
            default:
                break;
        }
    }

    branch(*this, mTimepoints, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_MIN());
}

TemporalConstraintNetwork::TemporalConstraintNetwork(bool shared, TemporalConstraintNetwork& other)
    : Gecode::Space(shared, other)
    , mVertices(other.mVertices)
{
    mTimepoints.update(*this, shared, other.mTimepoints);
}

TemporalConstraintNetwork* TemporalConstraintNetwork::nextSolution()
{
    Gecode::DFS<TemporalConstraintNetwork> searchEngine(this);
    TemporalConstraintNetwork* current = searchEngine.next();
    if(current)
    {
        return current;
    }
    return NULL;
}

bool TemporalConstraintNetwork::isConsistent(const temporal::QualitativeTemporalConstraintNetwork& tcn)
{
    if(tcn.getGraph()->order() <= 1)
    {
        return true;
    }

    TemporalConstraintNetwork* tcnSolver = new TemporalConstraintNetwork(tcn);
    TemporalConstraintNetwork* solution = tcnSolver->nextSolution();
    if(solution)
    {
        delete solution;
        return true;
    }

    delete solution;
    return false;
}

Gecode::Space* TemporalConstraintNetwork::copy(bool share)
{
    return new TemporalConstraintNetwork(share, *this);
}

std::vector<temporal::point_algebra::TimePoint::Ptr> TemporalConstraintNetwork::getSortedList(const temporal::QualitativeTemporalConstraintNetwork& tcn)
{
    std::vector<temporal::point_algebra::TimePoint::Ptr> timepoints = tcn.getTimepoints();
    sort(tcn, timepoints);
    return timepoints;
}

void TemporalConstraintNetwork::sort(const temporal::QualitativeTemporalConstraintNetwork& tcn, std::vector<temporal::point_algebra::TimePoint::Ptr>& timepoints)
{
    TemporalConstraintNetwork tcnSolver(tcn);
    TemporalConstraintNetwork* tcnSolution = tcnSolver.nextSolution();
    if(!tcnSolution)
    {
        throw std::invalid_argument("templ::solvers::csp::TemporalConstraintNetwork::getSortedList: cannot perform sorting. Network is inconsistent");
    }

    std::sort(timepoints.begin(), timepoints.end(), [tcnSolution](const temporal::point_algebra::TimePoint::Ptr& a, const temporal::point_algebra::TimePoint::Ptr& b){
            return tcnSolution->getValue(a) < tcnSolution->getValue(b);
            });

    delete tcnSolution;
}

uint32_t TemporalConstraintNetwork::getValue(const graph_analysis::Vertex::Ptr& v)
{
    return mTimepoints[ getVertexIdx(v) ].val();
}

size_t TemporalConstraintNetwork::getVertexIdx(const graph_analysis::Vertex::Ptr& vertex) const
{
    if(mVertices.empty())
    {
        throw std::runtime_error("templ::solvers::csp::TemporalConstraintNetwork::getVertexIdx: list of vertices is empty");
    }

    std::vector<graph_analysis::Vertex::Ptr>::const_iterator cit = std::find(mVertices.cbegin(), mVertices.cend(), vertex);

    if(cit != mVertices.cend())
    {
        return std::distance(mVertices.cbegin(), cit);
    }

    for(const Vertex::Ptr& v : mVertices)
    {
        if(!v)
        {
            std::cout << "Vertices:: n/a" << std::endl;
        } else {
            std::cout << "Vertices: " << v->toString() << std::endl;
        }
    }

    throw std::invalid_argument("tempL::solvers::csp::TemporalConstraintNetwork::getVertexIdx: given vertex '" + vertex->toString() + "' does not exist in the related graph");
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
