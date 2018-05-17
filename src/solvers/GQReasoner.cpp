#include "GQReasoner.hpp"
#include <templ/SharedPtr.hpp>
#include <utilmm/configfile/pkgconfig.hh>

using namespace graph_analysis;

namespace templ {
namespace solvers {

std::string GQReasoner::mDataPath = "";

GQReasoner::GQReasoner(const std::string& calculus, const graph_analysis::BaseGraph::Ptr& graph, const Edge::Ptr& edge)
    : mpGraph(graph->cloneEdges())
    , mpCalculus(0)
    , mpCalculus8r(0)
    , mpCalculus16r(0)
    , mpCalculus32r(0)
    , mpCalculusOp8r(0)
    , mpCalculusOp16r(0)
    , mpCalculusOp32r(0)
    , mpCSP8r(0)
    , mpCSP16r(0)
    , mpCSP32r(0)
    , mpSearch8r(0)
    , mpSearch16r(0)
    , mpSearch32r(0)
    , mpDefaultConstraintType(edge)
{
    init(calculus);
    translateGraph();
}

GQReasoner::~GQReasoner()
{
    delete mpSearch8r;
    delete mpSearch16r;
    delete mpSearch32r;

    delete mpCSP8r;
    delete mpCSP16r;
    delete mpCSP32r;

    delete mpCalculusOp8r;
    delete mpCalculusOp16r;
    delete mpCalculusOp32r;

    delete mpCalculus8r;
    delete mpCalculus16r;
    delete mpCalculus32r;

    delete mpCalculus;
}

bool GQReasoner::isConsistent(const temporal::QualitativeTemporalConstraintNetwork& qtcn)
{
    GQReasoner paReasoner("point", qtcn.getGraph(), make_shared<temporal::point_algebra::QualitativeTimePointConstraint>());
    if(paReasoner.getPrimarySolution() != graph_analysis::BaseGraph::Ptr())
    {
        return true;
    }
    return false;
}

void GQReasoner::init(const std::string& calculus)
{
    LOG_DEBUG_S << "Initializing GQReasoner for calculus '" << calculus << "'";

    try {
        utilmm::pkgconfig pkg("gqr");
        mDataPath = pkg.get("sharedir") + "/data";
    } catch(...)
    {
        throw std::runtime_error("templ::solvers::GQReasoner::init: GQR is not properly installed or your PKG_CONFIG_PATH is not properly set -- check 'pkg-config gqr'");
    }

    std::string calculusPath = mDataPath + "/" + calculus  + ".spec";
    std::ifstream stream;
    stream.open(calculusPath.c_str());
    CalculusReader calculusReader(calculus, mDataPath.c_str(), &stream);
    mpCalculus = calculusReader.makeCalculus();

    groundCalculus();

    if(mpCalculus8r)
    {
        LOG_DEBUG_S << "Initialize CalculusOp for <= 8 relations";
        mpCalculusOp8r = new CalculusOp8r_t(*mpCalculus8r);
        LOG_DEBUG_S << "Initialize CSP for <= 8 relations";
        mpCSP8r = new CSP8r_t(mpGraph->order(), *mpCalculusOp8r, "calculus_8r");
    } else if(mpCalculus16r)
    {
        LOG_DEBUG_S << "Initialize CalculusOp for <= 16 relations";
        mpCalculusOp16r = new CalculusOp16r_t(*mpCalculus16r);
        LOG_DEBUG_S << "Initialize CSP for <= 16 relations";
        mpCSP16r = new CSP16r_t(mpGraph->order(), *mpCalculusOp16r, "calculus_16r");
    } else if(mpCalculus32r)
    {
        LOG_DEBUG_S << "Initialize CalculusOp for <= 32 relations";
        mpCalculusOp32r = new CalculusOp32r_t(*mpCalculus32r);
        LOG_DEBUG_S << "Initialize CSP for <= 32 relations";
        mpCSP32r = new CSP32r_t(mpGraph->order(), *mpCalculusOp32r, "calculus_32r");
    }
}

void GQReasoner::translateGraph()
{
    LOG_DEBUG_S << "Translating graph: size: " << mpGraph->size() << ", order: " << mpGraph->order();
    using namespace graph_analysis;
    EdgeIterator::Ptr edgeIt = mpGraph->getEdgeIterator();
    while(edgeIt->next())
    {
        // hook: using getConstraintLabel from existing edge
        Edge::Ptr constraint = edgeIt->current();

        std::string constraintLabel = this->getConstraintLabel(constraint);
        if(constraintLabel == "P")
        {
            LOG_DEBUG_S << "Skipping encoding for P relation on : '" << constraint->toString() << "'";
            continue;
        }

        LOG_DEBUG_S << "Encode relation for constraint: '" << constraint->toString() << "'";

        assert(mpCalculus);
        if(!mpCalculus)
        {
            throw std::runtime_error("templ::solvers::GQReasoner::translateGraph: calculus is not initialized");
        }
        Relation relation = mpCalculus->encodeRelation(constraintLabel.c_str());

        if(mpCalculus8r)
        {
            GraphElementId sourceId = constraint->getSourceVertex()->getId(mpGraph->getId());
            GraphElementId targetId = constraint->getTargetVertex()->getId(mpGraph->getId());
            LOG_DEBUG_S << "Set constraint '" << constraint->toString() << "' to relation: '" << relation << "' -- (" << sourceId << " --> " << targetId << ")";
            mpCSP8r->setConstraint(sourceId, targetId, relation);
        } else if(mpCalculus16r)
        {
            mpCSP16r->setConstraint(constraint->getSourceVertex()->getId(mpGraph->getId()),
                constraint->getTargetVertex()->getId(mpGraph->getId()),relation);
        } else if(mpCalculus32r)
        {
            mpCSP32r->setConstraint(constraint->getSourceVertex()->getId(mpGraph->getId()),
                constraint->getTargetVertex()->getId(mpGraph->getId()),relation);
        }
    }

    LOG_DEBUG_S << "Translating graph completed";
}

BaseGraph::Ptr GQReasoner::getPrimarySolution()
{
    std::stringstream ss;
    if(mpCSP8r)
    {
        delete mpSearch8r;
        mpSearch8r = new DFSearch8r(*mpCSP8r, NULL);
        CSP8r_t* csp = mpSearch8r->run();
        if(!csp)
        {
            return BaseGraph::Ptr();
        }

        for (size_t i = 0; i < csp->getSize(); ++i)
        {
            for (size_t j = i; j < csp->getSize(); ++j)
            {
                LOG_DEBUG_S << "Update: " << i << "/" << j;
                Relation relation = csp->getConstraint(i, j).getRelation();

                std::string relationString = mpCalculus->relationToString(relation);
                size_t relationStringSize = relationString.size();
                assert(relationStringSize >= 4);
                // remove brackets and spaces ( < ) --> <
                std::string label = relationString.substr(2, relationStringSize -4);

                relabel(i,j, label);
                ss << i << " " << j << " " << label << std::endl;
            }
        }
    } else if(mpCSP16r)
    {
        delete mpSearch16r;
        mpSearch16r = new DFSearch16r(*mpCSP16r, NULL);
        CSP16r_t* csp = mpSearch16r->run();
        if(!csp)
        {
            return BaseGraph::Ptr();
        }

        for (size_t i = 0; i < csp->getSize(); ++i)
        {
            for (size_t j = i; j < csp->getSize(); ++j)
            {
                Relation relation = csp->getConstraint(i, j).getRelation();
                std::string relationString = mpCalculus->relationToString(relation);

                getOrCreateEdge(i,j)->setLabel(relationString);
            }
        }
    } else if(mpCSP32r)
    {
        delete mpSearch32r;
        mpSearch32r = new DFSearch32r(*mpCSP32r, NULL);
        CSP32r_t* csp = mpSearch32r->run();
        if(!csp)
        {
            return BaseGraph::Ptr();
        }

        for (size_t i = 0; i < csp->getSize(); ++i)
        {
            for (size_t j = i; j < csp->getSize(); ++j)
            {
                Relation relation = csp->getConstraint(i, j).getRelation();
                std::string relationString = mpCalculus->relationToString(relation);

                getOrCreateEdge(i,j)->setLabel(relationString);
            }
        }
    }

    mCurrentSolution  = ss.str();
    return mpGraph->cloneEdges();
}

graph_analysis::BaseGraph::Ptr GQReasoner::getNextSolution()
{
    std::stringstream ss;
    if(mpCSP8r)
    {
        if(!mpSearch8r)
        {
            throw std::runtime_error("templ::solvers::GQReasoner::getNextSolution: getPrimarySolution needs to called first");
        }

        CSP8r_t* csp = mpSearch8r->next();
        if(!csp)
        {
            return BaseGraph::Ptr();
        }

        for (size_t i = 0; i < csp->getSize(); i++)
        {
            for (size_t j = i; j < csp->getSize(); j++)
            {
                Relation relation = csp->getConstraint(i, j).getRelation();
                std::string relationString = mpCalculus->relationToString(relation);

                size_t relationStringSize = relationString.size();
                assert(relationStringSize >= 4);
                // remove brackets an spaces ( < ) --> <
                std::string label = relationString.substr(2, relationStringSize -4);
                relabel(i,j, label);
                ss << i << " " << j << " " << label << std::endl;
            }
        }
        delete csp;
    } else if(mpCSP16r)
    {
        if(!mpSearch16r)
        {
            throw std::runtime_error("templ::solvers::GQReasoner::getNextSolution: getPrimarySolution needs to called first");
        }

        CSP16r_t* csp = mpSearch16r->next();
        if(!csp)
        {
            return BaseGraph::Ptr();
        }

        for (size_t i = 0; i < csp->getSize(); i++)
        {
            for (size_t j = i; j < csp->getSize(); j++)
            {
                Relation relation = csp->getConstraint(i, j).getRelation();
                std::string relationString = mpCalculus->relationToString(relation);

                relabel(i,j,relationString);
            }
        }
        delete csp;
    } else if(mpCSP32r)
    {
        if(!mpSearch32r)
        {
            throw std::runtime_error("templ::solvers::GQReasoner::getNextSolution: getPrimarySolution needs to called first");
        }

        CSP32r_t* csp = mpSearch32r->next();
        if(!csp)
        {
            return BaseGraph::Ptr();
        }
        for (size_t i = 0; i < csp->getSize(); i++)
        {
            for (size_t j = i; j < csp->getSize(); j++)
            {
                Relation relation = csp->getConstraint(i, j).getRelation();
                std::string relationString = mpCalculus->relationToString(relation);

                relabel(i,j,relationString);
            }
        }
        delete csp;
    }

    mCurrentSolution = ss.str();
    return mpGraph->cloneEdges();
}

void GQReasoner::groundCalculus()
{
    if(mpCalculus->getNumberOfBaseRelations() <= 8)
    {
        LOG_DEBUG_S << "Grounding calculus with <= 8 relations";
        gqrtl::Relation8::init();
        mpCalculus8r = new gqrtl::CalculusOperations<gqrtl::Relation8>(*mpCalculus);
    } else if(mpCalculus->getNumberOfBaseRelations() <= 16)
    {
        LOG_DEBUG_S << "Grounding calculus with <= 16 relations";
        gqrtl::Relation16::init();
        mpCalculus16r = new gqrtl::CalculusOperations<gqrtl::Relation16>(*mpCalculus);
    } else if(mpCalculus->getNumberOfBaseRelations() <= 32)
    {
        LOG_DEBUG_S << "Grounding calculus with <= 32 relations";
        gqrtl::Relation32::init();
        mpCalculus32r = new gqrtl::CalculusOperations<gqrtl::Relation32>(*mpCalculus);
    } else {
        throw std::invalid_argument("templ::solvers::GQReasoner::groundCalculus: calculus with more than 32 base relations"
            " -- this is currently not supported (the core library GQR provides support though - see gqr_wrap.h)");
    }
}

Edge::Ptr GQReasoner::getOrCreateEdge(int i, int j) const
{
    Vertex::Ptr vertex_i = mpGraph->getVertex(i);
    Vertex::Ptr vertex_j = mpGraph->getVertex(j);

    std::vector<Edge::Ptr> edges = mpGraph->getEdges(vertex_i, vertex_j);
    if(edges.empty())
    {
        Edge::Ptr edge = mpDefaultConstraintType->clone();
        edge->setSourceVertex(vertex_i);
        edge->setTargetVertex(vertex_j);
        return edge;
    } else if(edges.size() == 1)
    {
        return edges[0];
    } else {
        throw std::runtime_error("templ::GQReasoner::getOrCreateEdge: more than one edge found between two vertices");
    }
}

void GQReasoner::relabel(int i, int j, const std::string& label)
{
    Edge::Ptr edge = getOrCreateEdge(i,j);

    // Cleanup residue from previous runs if necessary
    if(i != j)
    {
        Edge::Ptr inverseEdge = getOrCreateEdge(j,i);
        if(inverseEdge)
        {
            try {
                mpGraph->removeEdge(inverseEdge);
            } catch(...)
            {
                // ignore if not present
            }
        }
    }

    try {
        mpGraph->removeEdge(edge);
        LOG_DEBUG_S << "Removed edge: '" << edge->toString() << "' from graph: " << mpGraph->getId();
    } catch(...)
    {
        // ignore if not present -- then it needs to be added
    }
    Edge::Ptr relabeledEdge = edge->clone();
    relabeledEdge->setLabel(label);
    mpGraph->addEdge(relabeledEdge);
    LOG_DEBUG_S << "Added edge: '" << relabeledEdge->getSourceVertex()->toString() << " --> " << relabeledEdge->getTargetVertex()->toString() << " " << relabeledEdge->toString() << "' to graph: " << mpGraph->getId();
}


} // end namespace solvers
} // end namespace templ
