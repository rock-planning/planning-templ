#ifndef TEMPL_SOLVERS_GQ_REASONER_HPP
#define TEMPL_SOLVERS_GQ_REASONER_HPP

#include <graph_analysis/BaseGraph.hpp>
#include <gqr/libgqr.h>

namespace templ {
namespace solvers {

/**
 * \class GQReasoner
 * \brief Wrapper for the GQR reasoner
 * \see http://gki.informatik.uni-freiburg.de/events/aaai09-bench/tools/gqr-the-generic-qualitative-reasoner.html
 * \details This wrapper allow the usage of the Generic Qualitative Reasoner to
 * provide a set of valid solutions for partially defined temporal qualitiative networks
 * \verbatim
 using namespace templ::solvers::temporal::point_algebra;
 typedef QualitativeTimePointConstraint QTPC;
 QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

 QualitativeTimePoint::Ptr lastTp;
 int i = 0;
 for(; i < 15; ++i)
 {
     std::stringstream ss;
     ss << "t" << i;
     QualitativeTimePoint::Ptr tp(new QualitativeTimePoint(ss.str()));
     if(lastTp && i < 14)
     {
         tcn->addQualitativeConstraint(lastTp, tp, QTPC::Less);
         tcn->addQualitativeConstraint(tp, lastTp, QTPC::Greater);
     }

     lastTp = tp;
 }


 GQReasoner paReasoner("point", tcn->getGraph(), QTPC::Ptr(new QTPC()) );
 BaseGraph::Ptr primarySolution = paReasoner.getPrimarySolution();
 tcn->setConsistentNetwork(primarySolution);

 while(true)
 {
     BaseGraph::Ptr nextSolution = paReasoner.getNextSolution();

     EdgeIterator::Ptr edgeIt = nextSolution->getEdgeIterator();

     while(edgeIt->next())
     {
         Edge::Ptr edge = edgeIt->current();

         std::cout << "constraint: " << edge->getSourceVertex()->toString() << " " << edge->getLabel() << " "
                 << edge->getTargetVertex()->toString()) << std::endl;
     }
 }
 \endverbatim
 *
 */
class GQReasoner
{
public:
    /**
     * \param calculus
     * \param graph
     * \param defaultEdge
     */
    GQReasoner(const std::string& calculus, const graph_analysis::BaseGraph::Ptr& graph, const graph_analysis::Edge::Ptr& defaultEdge);
    virtual ~GQReasoner();

    /**
     * Get the primary (here: first) solution for the gq reasoner
     * \return A graph instance that shares the vertices with the graph that has
     * been used for initialization
     */
    graph_analysis::BaseGraph::Ptr getPrimarySolution();

    /*
     * Get the next solution for the gq reasoner
     * \return A graph instance that shares the vertices with the graph that has
     * been used for initialization
     */
    graph_analysis::BaseGraph::Ptr getNextSolution();

    /**
     * Get the current solution as string representation
     * \return A string that represents the current solution
     */
    std::string getCurrentSolutionString() const { return mCurrentSolution; }

protected:
    /**
     * Initialize the respective calculus, e.g. for point algebra use 'point'
     * Other (though untested) are: allen, cd, occ, opra1, opra2, opra4, rcc23
     * \see data folder of gqr installation for more available algbras
     */
    void init(const std::string& calculus);

    /**
     * Translate the directed graph into the internal representation, i.e.
     * the given graph is translated into the GQR CSP, so that there is a
     * starting point for the CSP
     */
    void translateGraph();

    /**
     * Return the constraint label of this edge -- if not overloaded will just
     * \return the label of the edge
     */
    virtual std::string getConstraintLabel(const graph_analysis::Edge::Ptr& e) const { assert(e); return e->getLabel(); }

private:
    void groundCalculus();

    /**
     * Retrieve the edge between i and j or create a new one
     * \return the edge found or the one newly created (via cloning from the
     * default constraint type)
     */
    graph_analysis::Edge::Ptr getOrCreateEdge(int i, int j) const;

    /**
     * Reset the constraint label between the edges i and j
     * Will remove any reverse edge j-i unless, j == i
     */
    void relabel(int i, int j, const std::string& label);

    graph_analysis::BaseGraph::Ptr mpGraph;
    static std::string msDataPath;
    Calculus* mpCalculus;
    gqrtl::CalculusOperations<gqrtl::Relation8>* mpCalculus8r;
    gqrtl::CalculusOperations<gqrtl::Relation16>* mpCalculus16r;
    gqrtl::CalculusOperations<gqrtl::Relation32>* mpCalculus32r;

    typedef gqrtl::CalculusOperations<gqrtl::Relation8> CalculusOp8r_t;
    typedef gqrtl::CalculusOperations<gqrtl::Relation16> CalculusOp16r_t;
    typedef gqrtl::CalculusOperations<gqrtl::Relation32> CalculusOp32r_t;

    typedef gqrtl::CSP<gqrtl::Relation8, CalculusOp8r_t > CSP8r_t;
    typedef gqrtl::CSP<gqrtl::Relation16, CalculusOp16r_t > CSP16r_t;
    typedef gqrtl::CSP<gqrtl::Relation32, CalculusOp32r_t > CSP32r_t;

    CalculusOp8r_t* mpCalculusOp8r;
    CalculusOp16r_t* mpCalculusOp16r;
    CalculusOp32r_t* mpCalculusOp32r;

    CSP8r_t* mpCSP8r;
    CSP16r_t* mpCSP16r;
    CSP32r_t* mpCSP32r;

    typedef gqrtl::DFS<gqrtl::Relation8> DFSearch8r;
    typedef gqrtl::DFS<gqrtl::Relation16> DFSearch16r;
    typedef gqrtl::DFS<gqrtl::Relation32> DFSearch32r;

    DFSearch8r* mpSearch8r;
    DFSearch16r* mpSearch16r;
    DFSearch32r* mpSearch32r;

    /// The default edge type
    graph_analysis::Edge::Ptr mpDefaultConstraintType;
    /// Current solution after relabeling
    std::string mCurrentSolution;
};

} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_GQ_REASONER_HPP
