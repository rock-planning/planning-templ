#include "TemporalConstraintNetwork.hpp"
#include "../../solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp"
#include <base-logging/Logging.hpp>

using namespace graph_analysis;
using namespace templ::solvers::temporal;

namespace templ {
namespace solvers {
namespace csp {

TemporalConstraintNetworkBase::TemporalConstraintNetworkBase()
{}

TemporalConstraintNetworkBase::TemporalConstraintNetworkBase(const temporal::QualitativeTemporalConstraintNetwork& tcn)
    : mVertices(tcn.getGraph()->getAllVertices())
{}

TemporalConstraintNetworkBase::TemporalConstraintNetworkBase(const temporal::QualitativeTemporalConstraintNetwork& tcn,
            Gecode::Space& space,
            Gecode::IntVarArray& timepoints)
    : mVertices(tcn.getGraph()->getAllVertices())
{
    addConstraints(tcn, space, timepoints);
}

TemporalConstraintNetworkBase::~TemporalConstraintNetworkBase()
{}

void TemporalConstraintNetworkBase::addConstraints(const temporal::QualitativeTemporalConstraintNetwork& tcn,
        Gecode::Space& space,
        Gecode::IntVarArray& timepoints)
{
    EdgeIterator::Ptr edgeIt = tcn.getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {
        point_algebra::QualitativeTimePointConstraint::Ptr constraint = dynamic_pointer_cast<point_algebra::QualitativeTimePointConstraint>(edgeIt->current());
        if(!constraint)
        {
            throw std::runtime_error("templ::solvers::csp::TemporalConstraintNetwork: encountered an edge which is not a QualitativeTimePointConstraint");
        }

        size_t sourceIdx = getVertexIdx( constraint->getSourceVertex(), mVertices );
        size_t targetIdx = getVertexIdx( constraint->getTargetVertex(), mVertices );

        LOG_DEBUG_S << "Add constraint: " << point_algebra::QualitativeTimePointConstraint::TypeTxt[constraint->getType() ] << " from: " << sourceIdx << " to: " << targetIdx << std::endl
            << constraint->getSourceVertex()->toString() << " --> " << constraint->getTargetVertex()->toString();


        switch(constraint->getType())
        {
            case point_algebra::QualitativeTimePointConstraint::Empty:
                space.fail();
                break;
            case point_algebra::QualitativeTimePointConstraint::Less:
                Gecode::rel(space, timepoints[sourceIdx], Gecode::IRT_LE, timepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::LessOrEqual:
                Gecode::rel(space, timepoints[sourceIdx], Gecode::IRT_LQ, timepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::Greater:
                Gecode::rel(space, timepoints[sourceIdx], Gecode::IRT_GR, timepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::GreaterOrEqual:
                Gecode::rel(space, timepoints[sourceIdx], Gecode::IRT_GQ, timepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::Equal:
                Gecode::rel(space, timepoints[sourceIdx], Gecode::IRT_EQ, timepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::Distinct:
                Gecode::rel(space, timepoints[sourceIdx], Gecode::IRT_NQ, timepoints[targetIdx]);
                break;
            case point_algebra::QualitativeTimePointConstraint::Universal:
                break;
            default:
                break;
        }
    }
}

bool TemporalConstraintNetworkBase::isConsistent(const temporal::QualitativeTemporalConstraintNetwork& tcn)
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



std::vector<temporal::point_algebra::TimePoint::Ptr> TemporalConstraintNetworkBase::getSortedList(const temporal::QualitativeTemporalConstraintNetwork& tcn)
{
    std::vector<temporal::point_algebra::TimePoint::Ptr> timepoints = tcn.getTimepoints();
    sort(tcn, timepoints);
    return timepoints;
}

void TemporalConstraintNetworkBase::sort(const temporal::QualitativeTemporalConstraintNetwork& tcn,
        std::vector<temporal::point_algebra::TimePoint::Ptr>& timepoints)
{
    TemporalConstraintNetwork tcnSolver(tcn);
    TemporalConstraintNetwork* tcnSolution = tcnSolver.nextSolution();
    if(!tcnSolution)
    {
        throw std::invalid_argument("templ::solvers::csp::TemporalConstraintNetwork::getSortedList: cannot perform sorting. Network is inconsistent");
    }

    sort(tcnSolution->mTimepoints, timepoints, tcnSolver.mVertices);

    delete tcnSolution;
}
void TemporalConstraintNetworkBase::sort(const Gecode::IntVarArray& timepointArray,
        std::vector<temporal::point_algebra::TimePoint::Ptr>& timepoints,
        graph_analysis::Vertex::PtrList& verticesCache)

{
    if(!timepointArray.assigned())
    {
        throw std::invalid_argument("templ::solvers::csp::TemporalConstraintNetwork::sort: cannot perform sorting. Network is not fully assigned");
    }

    std::sort(timepoints.begin(), timepoints.end(), [&timepointArray, &verticesCache](const temporal::point_algebra::TimePoint::Ptr& a, const temporal::point_algebra::TimePoint::Ptr& b){
            return TemporalConstraintNetworkBase::getValue(a, timepointArray, verticesCache) < TemporalConstraintNetworkBase::getValue(b, timepointArray, verticesCache);
            });
}

uint32_t TemporalConstraintNetworkBase::getValue(const graph_analysis::Vertex::Ptr& v,
        const Gecode::IntVarArray& timepoints,
        graph_analysis::Vertex::PtrList& verticesCache)
{
    const Gecode::IntVar& var = timepoints[ getVertexIdx(v, verticesCache) ];
    if(var.assigned())
    {
        return var.val();
    }
    throw std::invalid_argument("templ::solvers::csp::TemporalConstraintNetwork::getValue: constraint variable corresponding to the given vertex has not yet been assigned");
}

size_t TemporalConstraintNetworkBase::getVertexIdx(const graph_analysis::Vertex::Ptr& vertex,
            graph_analysis::Vertex::PtrList& verticesCache
        )
{
    if(verticesCache.empty())
    {
        throw std::runtime_error("templ::solvers::csp::TemporalConstraintNetwork::getVertexIdx: list of vertices is empty");
    }

    std::vector<graph_analysis::Vertex::Ptr>::const_iterator cit = std::find(verticesCache.cbegin(), verticesCache.cend(), vertex);

    if(cit != verticesCache.cend())
    {
        return std::distance(verticesCache.cbegin(), cit);
    }

    for(const Vertex::Ptr& v : verticesCache)
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


temporal::QualitativeTemporalConstraintNetwork::Ptr TemporalConstraintNetworkBase::translate(const Gecode::IntVarArray& solution)
{
    temporal::QualitativeTemporalConstraintNetwork::Ptr qtcn = make_shared<temporal::QualitativeTemporalConstraintNetwork>();

    namespace pa = temporal::point_algebra;

    for(size_t a = 0; a < mVertices.size() - 1; ++a)
    {
        const pa::TimePoint::Ptr& vertexA = dynamic_pointer_cast<pa::TimePoint>(mVertices[a]);
        uint32_t valueA = getValue(vertexA, solution, mVertices);
        for(size_t b = a + 1; b < mVertices.size(); ++b)
        {
            const pa::TimePoint::Ptr& vertexB = dynamic_pointer_cast<pa::TimePoint>(mVertices[b]);
            uint32_t valueB = getValue(vertexB, solution, mVertices);

            pa::QualitativeTimePointConstraint::Type type = pa::QualitativeTimePointConstraint::Empty;

            if(valueA == valueB)
            {
                type = pa::QualitativeTimePointConstraint::Equal;
            } else if(valueA < valueB)
            {
                type = pa::QualitativeTimePointConstraint::Less;
            } else // valueA > valueB
            {
                type = pa::QualitativeTimePointConstraint::Greater;
            }

            qtcn->addQualitativeConstraint(vertexA, vertexB, type);
        }
    }
    return qtcn;
}

TemporalConstraintNetwork::TemporalConstraintNetwork(const temporal::QualitativeTemporalConstraintNetwork& tcn)
    : Gecode::Space()
    , TemporalConstraintNetworkBase(tcn)
    , mTimepoints(*this, mVertices.size(), 0, mVertices.size() - 1)
{

    addConstraints(tcn, *this, mTimepoints);
    branch(*this, mTimepoints, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_MIN());
}

TemporalConstraintNetwork::TemporalConstraintNetwork(bool shared, TemporalConstraintNetwork& other)
    : Gecode::Space(shared, other)
{
    setVertices( other.getVertices() );
    mTimepoints.update(*this, shared, other.mTimepoints);
}

TemporalConstraintNetwork::~TemporalConstraintNetwork()
{}


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

Gecode::Space* TemporalConstraintNetwork::copy(bool share)
{
    return new TemporalConstraintNetwork(share, *this);
}

temporal::TemporalConstraintNetwork::Ptr TemporalConstraintNetwork::getTemporalConstraintNetwork() const
{
    temporal::TemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    return tcn;
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
