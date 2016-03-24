#include <boost/test/unit_test.hpp>
#include <templ/solvers/csp/TemporalCSP.hpp>
#include "../test_utils.hpp"

#include <gqr/gqr_wrap.h>
#include <gqr/libgqr.h>
#include <gqr/RestartsFramework.h>
#include <gqr/gqrtl/RestartingDFS.h>

#include <templ/solvers/GQReasoner.hpp>

#include <utilmm/configfile/pkgconfig.hh>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>

using namespace templ;
using namespace templ::solvers::csp;
using namespace templ::solvers::temporal;
using namespace templ::solvers::temporal::point_algebra;

typedef QualitativeTimePointConstraint QTPC;

static void print_csp(GqrCsp* csp)
{
    char* name = gqr_csp_get_name(csp);
    printf("%s\n", name);
    free(name);

    int i, j;
    for (i = 0; i < gqr_csp_get_size(csp); i++) {
            for (j = i; j < gqr_csp_get_size(csp); j++) {
                    char* str = gqr_csp_get_constraint(csp, i, j);
                    printf("\t%d, %d\t%s\n", i, j, str);
                    free(str);
            }
    }
}


BOOST_AUTO_TEST_SUITE(temporal_csp)

BOOST_AUTO_TEST_CASE(gqr)
{
    std::string path;
    try {
        utilmm::pkgconfig pkg("gqr");
        path = pkg.get("sharedir") + "/data";
    } catch(...)
    {
        BOOST_REQUIRE_MESSAGE(false, "GQR is not properly installed, cannot find calculus");
    }
#if !GLIB_CHECK_VERSION(2,35,0)
    g_type_init(); /* gobject init (required!) */
#endif

    /* load Allen's Interval calculus */
    GqrCalculus* allen = gqr_calculus_new("allen", path.c_str());

    if (allen == NULL)
    {
        BOOST_REQUIRE_MESSAGE(false,"Could not load allen calculus. Tried \"data/allen.spec\"");
    }

    /* generate new csp */
    GqrCsp* csp = gqr_csp_new(4, allen);
    assert(csp != NULL);

    gqr_csp_set_name(csp, "Example CSP");

    /* add some constraints */
    gqr_csp_add_constraint(csp, 0, 1, "( < )");
    gqr_csp_add_constraint(csp, 2, 4, "( m )");
    gqr_csp_add_constraint(csp, 2, 3, "( di )");

    print_csp(csp);

    /* establish path consistency */
    GqrSolver* solver = gqr_solver_new(allen);

    bool ret = gqr_solver_enforce_algebraic_closure(solver, csp);
    assert(ret); /* the example is path consistent */

    /* get and output a scenario of 'csp' */
    GqrCsp* res = gqr_solver_get_scenario(solver, csp);
    assert(res); /* the example is satisfiable */

    printf("One scenario is:\n");
    print_csp(res);

    /* clean up */
    gqr_solver_unref(solver);
    gqr_csp_unref(res);
    gqr_csp_unref(csp);
    gqr_calculus_unref(allen);

//    GQR_Calculus allen;
//
//
//    BOOST_REQUIRE_MESSAGE(allen.set_calculus("allen", path), "Set allen calculus");
//
//    GQR_CSP unsat(3, allen);
//    unsat.set_constraint(0, 1, "( < )");
//    unsat.set_constraint(1, 2, "( < )");
//    unsat.set_constraint(0, 2, "( = )");
//
//    GQR_CSP cpy(unsat);
//
//    GQR_Solver solver(allen);
//
//    BOOST_REQUIRE_MESSAGE(solver.enforce_algebraic_closure(unsat) ==  false, "Enforce algebraic closure: false");
//    assert(unsat.check_refines(cpy)); //, "CHECK REFINES");
//
//    BOOST_REQUIRE_MESSAGE(solver.get_scenario(unsat) == NULL, "Cannot be satisfied");
//    BOOST_REQUIRE_MESSAGE(solver.get_scenario(cpy) == NULL, "Cannot be satisfied");
//
//    GQR_CSP sat(5, allen);
//    sat.set_constraint(0,1, "( < )");
//    sat.set_constraint(2,3, "( < )");
//    sat.set_constraint(3,4, "( < )");
}

BOOST_AUTO_TEST_CASE(gqr_cxx_api)
{
    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr lastTp;
    int i = 0;
    for(; i < 30; ++i)
    {
        std::stringstream ss;
        ss << "t" << i;
        QualitativeTimePoint::Ptr tp(new QualitativeTimePoint(ss.str()));
        if(lastTp)
        {
            tcn->addQualitativeConstraint(lastTp, tp, QTPC::Greater);
            tcn->addQualitativeConstraint(tp, lastTp, QTPC::Less);
        }

        lastTp = tp;
    }

    std::string path;
    try {
        utilmm::pkgconfig pkg("gqr");
        path = pkg.get("sharedir") + "/data";
    } catch(...)
    {
        BOOST_REQUIRE_MESSAGE(false, "GQR is not properly installed, cannot find calculus");
    }

    Calculus* pointAlgebra;
    gqrtl::CalculusOperations<gqrtl::Relation8>* pointAlgebraOp;

    std::string calculusPath = path + "/point.spec";
    std::ifstream stream_r;
    stream_r.open(calculusPath.c_str());

    CalculusReader paCalculusReader("point", path.c_str(), &stream_r);
    pointAlgebra = paCalculusReader.makeCalculus();
    pointAlgebraOp = new gqrtl::CalculusOperations<gqrtl::Relation8>(*pointAlgebra);

    typedef gqrtl::CSP<gqrtl::Relation8, gqrtl::CalculusOperations<gqrtl::Relation8> > CSPSimple;

    BOOST_TEST_MESSAGE("ORDER of graph: " << tcn->getGraph()->order());
    CSPSimple csp(tcn->getGraph()->order() + 1, *pointAlgebraOp, "test");

    using namespace graph_analysis;

    EdgeIterator::Ptr edgeIt = tcn->getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {
        point_algebra::QualitativeTimePointConstraint::Ptr constraint = dynamic_pointer_cast<point_algebra::QualitativeTimePointConstraint>(edgeIt->current());
        point_algebra::QualitativeTimePointConstraint::Type type = constraint->getType();
        /* add some constraints */
        std::string constraint_label = QualitativeTimePointConstraint::TypeTxt[type];
        //std::string constraint_label = "< = >";
        BOOST_TEST_MESSAGE("Adding edge" << constraint->toString() << " --- label: " << constraint_label);
        csp.setConstraint(constraint->getSourceVertex()->getId(tcn->getGraph()->getId()),
                constraint->getTargetVertex()->getId(tcn->getGraph()->getId()), pointAlgebra->encodeRelation(constraint_label.c_str()));

        //csp.setConstraint(constraint->getTargetVertex()->getId(tcn->getGraph()->getId()),
        //        constraint->getSourceVertex()->getId(tcn->getGraph()->getId()),
        //        pointAlgebra->encodeRelation(constraint_label.c_str()));

    }

    for (size_t i = 0; i < tcn->getGraph()->order(); i++)
    {
        for (size_t j = i; j < tcn->getGraph()->order(); j++)
        {
            Relation relation = csp.getConstraint(i, j).getRelation();
            std::string relationString = pointAlgebra->relationToString(relation);

            BOOST_TEST_MESSAGE("Relation for " << i << " -- " << j << " " << relation << ": " << relationString);
            //result->csp->setConstraint(i, j, );
        }
    }

    gqrtl::DFS<gqrtl::Relation8> search(csp, NULL);
    CSPSimple* res = search.run();
    for (size_t i = 0; i < res->getSize(); i++)
    {
        for (size_t j = i; j < res->getSize(); j++)
        {
            Relation relation = res->getConstraint(i, j).getRelation();
            std::string relationString = pointAlgebra->relationToString(relation);

            BOOST_TEST_MESSAGE("Relation for " << i << " -- " << j << " " << relation << ": " << relationString);
        }
    }
    assert(res);
    delete res;

    for(int i = 0; i < 100; ++i)
    {
        try {
            res = search.next();
        } catch(const std::invalid_argument& e)
        {
            BOOST_TEST_MESSAGE("Search aborted on iteration: " << i);
            break;
        }
        if(res)
        {
            for (size_t i = 0; i < res->getSize(); i++)
            {
                for (size_t j = i; j < res->getSize(); j++)
                {
                    Relation relation = res->getConstraint(i, j).getRelation();
                    std::string relationString = pointAlgebra->relationToString(relation);

                    BOOST_TEST_MESSAGE("Relation for " << i << " -- " << j << " " << relation << ": " << relationString);
                    //result->csp->setConstraint(i, j, );
                }
            }
            delete res;
        } else {
            BOOST_TEST_MESSAGE("Search aborted on iteration: " << i << " since result was NULL");
            break;
        }
    }
}

BOOST_AUTO_TEST_CASE(gqr_reasoner)
{
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

    using namespace graph_analysis;
    using namespace templ::solvers;
    {
        EdgeIterator::Ptr edgeIt = tcn->getGraph()->getEdgeIterator();
        while(edgeIt->next())
        {
            Edge::Ptr edge = edgeIt->current();

            BOOST_TEST_MESSAGE("    constraint: " << edge->getSourceVertex()->toString() << " " << edge->getLabel() << " "
                    << edge->getTargetVertex()->toString());
        }
    }

    GQReasoner paReasoner("point", tcn->getGraph(), QTPC::Ptr(new QTPC()) );
    BaseGraph::Ptr primarySolution = paReasoner.getPrimarySolution();
    {
        BOOST_REQUIRE_MESSAGE(primarySolution, "Found primary solution size: " << primarySolution->size());
        EdgeIterator::Ptr edgeIt = primarySolution->getEdgeIterator();
        while(edgeIt->next())
        {
            Edge::Ptr edge = edgeIt->current();

            BOOST_TEST_MESSAGE("    constraint: " << edge->getSourceVertex()->toString() << " " << edge->getLabel() << " "
                    << edge->getTargetVertex()->toString());
        }
    }

    tcn->setGraph(primarySolution);
    BOOST_REQUIRE_MESSAGE( tcn->isConsistent(), "Primary solution consistency checked with PA incremental algo");

    point_algebra::TimePointComparator tpc(tcn);

    VertexIterator::Ptr vertexOutIt = tcn->getGraph()->getVertexIterator();
    while(vertexOutIt->next())
    {
        TimePoint::Ptr o_vertex = dynamic_pointer_cast<TimePoint>(vertexOutIt->current());

        VertexIterator::Ptr vertexInIt = tcn->getGraph()->getVertexIterator();
        while(vertexInIt->next())
        {
            TimePoint::Ptr i_vertex = dynamic_pointer_cast<TimePoint>(vertexInIt->current());

            if(tpc.equals(i_vertex, o_vertex))
            {
                BOOST_TEST_MESSAGE("Equal: " << i_vertex->toString() << " == " << o_vertex->toString());
            } else if(tpc.lessThan(i_vertex, o_vertex))
            {
                BOOST_TEST_MESSAGE("LessThan: " << i_vertex->toString() << " < " << o_vertex->toString());
            } else if(tpc.greaterThan(i_vertex, o_vertex))
            {
                BOOST_TEST_MESSAGE("GreaterThan: " << i_vertex->toString() << " > " << o_vertex->toString());
            }
        }
    }

    while(true)
    {
        BaseGraph::Ptr nextSolution = paReasoner.getNextSolution();
        if(!nextSolution)
        {
            BOOST_TEST_MESSAGE("Found no solution");
            break;
        }

        BOOST_TEST_MESSAGE("Found another solution");
        EdgeIterator::Ptr edgeIt = nextSolution->getEdgeIterator();

        while(edgeIt->next())
        {
            Edge::Ptr edge = edgeIt->current();

            BOOST_TEST_MESSAGE("    constraint: " << edge->getSourceVertex()->toString() << " " << edge->getLabel() << " "
                    << edge->getTargetVertex()->toString());
        }
    }
}

BOOST_AUTO_TEST_CASE(gqr_c_api)
{
    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr lastTp;
    int i = 0;
    for(; i < 4; ++i)
    {
        std::stringstream ss;
        ss << "t" << i;
        QualitativeTimePoint::Ptr tp(new QualitativeTimePoint(ss.str()));
        if(lastTp)
        {
            tcn->addQualitativeConstraint(lastTp, tp, QTPC::Less);
            tcn->addQualitativeConstraint(tp, lastTp, QTPC::Greater);
        }

        lastTp = tp;
    }

    std::string path;
    try {
        utilmm::pkgconfig pkg("gqr");
        path = pkg.get("sharedir") + "/data";
    } catch(...)
    {
        BOOST_REQUIRE_MESSAGE(false, "GQR is not properly installed, cannot find calculus");
    }
#if !GLIB_CHECK_VERSION(2,35,0)
    g_type_init(); /* gobject init (required!) */
#endif

    GqrCalculus* calculus = gqr_calculus_new("point", path.c_str());
    if (calculus == NULL)
    {
        BOOST_REQUIRE_MESSAGE(false,"Could not load point calculus. Tried \"data/point.spec\"");
    }

    using namespace graph_analysis;
    std::vector<Vertex::Ptr> vertices = tcn->getGraph()->getAllVertices();

    /* generate new csp */
    GqrCsp* csp = gqr_csp_new(vertices.size(), calculus);
    assert(csp != NULL);
    gqr_csp_set_name(csp, "Test CSP");

    EdgeIterator::Ptr edgeIt = tcn->getGraph()->getEdgeIterator();
    while(edgeIt->next())
    {
        point_algebra::QualitativeTimePointConstraint::Ptr constraint = dynamic_pointer_cast<point_algebra::QualitativeTimePointConstraint>(edgeIt->current());
        point_algebra::QualitativeTimePointConstraint::Type type = constraint->getType();
        /* add some constraints */
        std::string constraint_label = "( " + QualitativeTimePointConstraint::TypeTxt[type] +" )";
        BOOST_TEST_MESSAGE("Adding edge" << constraint->toString() << " --- label: " << constraint_label);
        gqr_csp_add_constraint(csp, constraint->getSourceVertex()->getId(tcn->getGraph()->getId()),
                constraint->getTargetVertex()->getId(tcn->getGraph()->getId()), constraint_label.c_str());
    }
    print_csp(csp);

    /* establish path consistency */
    GqrSolver* solver = gqr_solver_new(calculus);

    bool ret = gqr_solver_enforce_algebraic_closure(solver, csp);
    BOOST_REQUIRE_MESSAGE(ret, "Example is path consistent");

    /* get and output a scenario of 'csp' */
    GqrCsp* res = gqr_solver_get_scenario(solver, csp);
    assert(res); /* the example is satisfiable */

    printf("One scenario is:\n");
    print_csp(res);
    gqr_csp_unref(res);

    /* clean up */
    gqr_solver_unref(solver);
    gqr_csp_unref(csp);
    gqr_calculus_unref(calculus);

}

BOOST_AUTO_TEST_CASE(start)
{

    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr lastTp;
    int i = 0;
    for(; i < 10; ++i)
    {
        std::stringstream ss;
        ss << "t" << i;
        QualitativeTimePoint::Ptr tp(new QualitativeTimePoint(ss.str()));
        if(lastTp)
        {
            tcn->addQualitativeConstraint(lastTp, tp, QTPC::Greater);
            tcn->addQualitativeConstraint(tp, lastTp, QTPC::Less);
        }

        lastTp = tp;
    }

    TemporalCSP tcsp(tcn);
    BOOST_REQUIRE_MESSAGE(tcsp.nextSolution(), "TCSP has a solution");
}

BOOST_AUTO_TEST_SUITE_END()
