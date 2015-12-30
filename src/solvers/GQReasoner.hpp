#ifndef TEMPL_SOLVERS_GQ_REASONER_HPP
#define TEMPL_SOLVERS_GQ_REASONER_HPP

#include <graph_analysis/BaseGraph.hpp>
//#include <gqr/gqr_wrap.h>
#include <gqr/libgqr.h>
//#include <gqr/RestartsFramework.h>
//#include <gqr/gqrtl/RestartingDFS.h>

namespace templ {
namespace solvers {

class GQReasoner
{
public:
    GQReasoner(const std::string& calculus, const graph_analysis::BaseGraph::Ptr& graph, const graph_analysis::Edge::Ptr& defaultEdge);
    virtual ~GQReasoner();

    graph_analysis::BaseGraph::Ptr getPrimarySolution();

    graph_analysis::BaseGraph::Ptr getNextSolution();

    /**
     * Get the current solution as string representation
     */
    std::string getCurrentSolutionString() const { return mCurrentSolution; }

protected:
    void init(const std::string& calculus);

    /**
     * Translate the directed graph into the internal representation
     */
    void translateGraph();

    /**
     * Return the constraint label of this edge -- if not overloaded will just
     * return the label of the edge
     */
    virtual std::string getConstraintLabel(const graph_analysis::Edge::Ptr& e) const { assert(e); return e->getLabel(); }

private:
    void groundCalculus();

    graph_analysis::Edge::Ptr getEdge(int i, int j) const;

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
