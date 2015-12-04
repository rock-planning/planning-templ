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

BOOST_AUTO_TEST_CASE(start)
{

    QualitativeTemporalConstraintNetwork::Ptr tcn(new QualitativeTemporalConstraintNetwork());

    QualitativeTimePoint::Ptr lastTp;
    int i = 0;
    for(; i < 100; ++i)
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
