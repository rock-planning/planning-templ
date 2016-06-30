#include <numeric/Combinatorics.hpp>

#include "TemporalCSP.hpp"
#include "TemporalCSPPropagators.hpp"

#include <gecode/minimodel.hh>
#include <gecode/minimodel.hh>

#include <base-logging/Logging.hpp>

using namespace templ::solvers::temporal;

namespace templ {
namespace solvers {
namespace csp {

TemporalCSP::TemporalCSP(const QualitativeTemporalConstraintNetwork::Ptr& tcn)
    : Gecode::Space()
    , mTCN(tcn)
    , mTemporalRelations(*this, tcn->getGraph()->order()*tcn->getGraph()->order(), -1, 1)
{
    using namespace graph_analysis;
    using namespace point_algebra;

    LOG_WARN_S << "INIT";

    std::vector<Vertex::Ptr> vertices = tcn->getGraph()->getAllVertices();
    std::map<Vertex::Ptr, size_t> vertexToIndex;

    Gecode::Matrix<Gecode::IntVarArray> temporalConstraints(mTemporalRelations, vertices.size(), vertices.size());

    std::vector<size_t> indices;
    size_t i = 0;
    for(; i < vertices.size(); ++i)
    {
        Vertex::Ptr i_vertex = vertices[i];
        vertexToIndex[i_vertex] = i;

        indices.push_back(i);
        for(size_t j = 0; j < vertices.size(); ++j)
        {
            if(i == j)
            {
                continue;
            }

            Vertex::Ptr j_vertex = vertices[j];
            vertexToIndex[j_vertex] = j;

            point_algebra::QualitativeTimePointConstraint::Ptr constraint = tcn->getBidirectionalConstraint(i_vertex, j_vertex);
            point_algebra::QualitativeTimePointConstraint::Type type = constraint->getType();

            Gecode::IntVar var = temporalConstraints(i,j);
            // -1 -> LESS
            // 0 -> EQUAL
            // 1 -> GREATER
            int less = -1;
            int equal = 0;
            int greater = 1;

            using namespace Gecode;
            switch(type)
            {
                case QualitativeTimePointConstraint::Empty:
                    throw std::runtime_error("TemporalCSP: inconsistent network given as input");
                case QualitativeTimePointConstraint::Less:
                    rel(*this, var, IRT_EQ, less);
                    break;
                case QualitativeTimePointConstraint::Greater:
                    rel(*this, var, IRT_EQ, greater);
                    break;
                case QualitativeTimePointConstraint::Equal:
                    rel(*this, var, IRT_EQ, equal);
                    break;
                case QualitativeTimePointConstraint::LessOrEqual:
                    rel(*this, var, IRT_NQ, greater);
                    break;
                case QualitativeTimePointConstraint::GreaterOrEqual:
                    rel(*this, var, IRT_NQ, less);
                    break;
                case QualitativeTimePointConstraint::Universal:
                    // this is the default
                    break;
                case QualitativeTimePointConstraint::Distinct:
                default:
                    break;

            }
            Gecode::IntVar var_sym = temporalConstraints(j,i);
            //LOG_WARN_S << "POST_SYMMETRIC";
            // symmetric constraint
            pa_symmetric(*this, var, var_sym);
            //LOG_WARN_S << "DONE_SYMMETRIC";
        }
    }

    LOG_WARN_S << "START TRIANGLES";
    ///std::vector<Edge::Ptr> edges = tcn->getGraph()->getAllEdges();
    ///std::vector<Edge::Ptr>::const_iterator oit = edges.begin();
    ///for(; oit != edges.end(); ++oit)
    ///{
    ///    const Edge::Ptr& oEdge = *oit;
    ///    std::vector<Edge::Ptr>::const_iterator iit = edges.begin();
    ///    for(; iit != edges.end(); ++iit)
    ///    {
    ///        const Edge::Ptr& iEdge = *iit;
    ///        bool oSourceEqualsISource = oEdge->getSourceVertex() == iEdge->getSourceVertex();
    ///        bool oSourceEqualsITarget = oEdge->getSourceVertex() == iEdge->getTargetVertex();
    ///        bool oTargetEqualsISource = oEdge->getTargetVertex() == iEdge->getSourceVertex();
    ///        bool oTargetEqualsITarget = oEdge->getTargetVertex() == iEdge->getTargetVertex();
    ///        if((oSourceEqualsISource || oSourceEqualsITarget) && (oTargetEqualsISource || oTargetEqualsISource))
    ///        {
    ///            continue; // bidirectional edge
    ///        } if((oSourceEqualsISource || oSourceEqualsITarget) || (oTargetEqualsISource || oTargetEqualsISource))
    ///        {
    ///            size_t i,j,k;
    ///            // triangle found
    ///            if(oSourceEqualsISource)
    ///            {
    ///                i = vertexToIndex[oEdge->getSourceVertex()];
    ///                j = vertexToIndex[oEdge->getTargetVertex()];
    ///                k = vertexToIndex[iEdge->getTargetVertex()];
    ///            } else if(oTargetEqualsITarget) {
    ///                i = vertexToIndex[iEdge->getSourceVertex()];
    ///                j = vertexToIndex[oEdge->getSourceVertex()];
    ///                k = vertexToIndex[iEdge->getTargetVertex()];
    ///            } else if (oSourceEqualsITarget)
    ///            {
    ///                i = vertexToIndex[iEdge->getSourceVertex()];
    ///                j = vertexToIndex[iEdge->getTargetVertex()];
    ///                k = vertexToIndex[oEdge->getTargetVertex()];
    ///            } else if(oTargetEqualsISource)
    ///            {
    ///                i = vertexToIndex[oEdge->getSourceVertex()];
    ///                j = vertexToIndex[oEdge->getTargetVertex()];
    ///                k = vertexToIndex[iEdge->getTargetVertex()];
    ///            }

    ///            Gecode::IntVar var_ik = temporalConstraints(i,k);
    ///            Gecode::IntVar var_ij = temporalConstraints(i,j);
    ///            Gecode::IntVar var_jk = temporalConstraints(j,k);
    ///            path_consistent(*this, var_ik, var_ij, var_jk);
    ///        }

    ///    }
    ///}

    numeric::Combination<size_t> triangles(indices, 3, numeric::EXACT);
    do {
        std::vector<size_t> triangle = triangles.current();
        Gecode::IntVar var_ik = temporalConstraints(triangle[0], triangle[2]);
        Gecode::IntVar var_ij = temporalConstraints(triangle[0], triangle[1]);
        Gecode::IntVar var_jk = temporalConstraints(triangle[1], triangle[2]);

        LOG_WARN_S << "POST_PATH CONSISTENCY";
        path_consistent(*this, var_ik, var_ij, var_jk);
    } while(triangles.next());

    // for e
    branch(*this, mTemporalRelations, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_SPLIT_MIN());
    branch(*this, mTemporalRelations, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_SPLIT_MIN());
    branch(*this, mTemporalRelations, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_SPLIT_MIN());
}

TemporalCSP::TemporalCSP(bool share, TemporalCSP& other)
    : Gecode::Space(share, other)
{
}

Gecode::Space* TemporalCSP::copy(bool share)
{
    return new TemporalCSP(share, *this);
}

TemporalCSP::SolutionList TemporalCSP::solve(const QualitativeTemporalConstraintNetwork::Ptr& tcn)
{
    return SolutionList();
}

TemporalCSP* TemporalCSP::nextSolution()
{
    LOG_WARN_S << "START SOLUTION";
    Gecode::BAB<TemporalCSP> searchEngine(this);
    //Gecode::DFS<TemporalCSP> searchEngine(this);

    TemporalCSP* current = searchEngine.next();
    if(current == NULL)
    {
        throw std::runtime_error("templ::solvers::csp::ModelDistribution::solve: no solution found");
    } else {
        return current;
    }
}

TemporalCSP::Solution TemporalCSP::getSolution()
{
    throw std::runtime_error("NOT IMPLEMENTED: TemporalCSP::getSolution");
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
