#include <iostream>
#include <sstream>
#include <numeric/Stats.hpp>
#include <base/Time.hpp>

#include <templ/solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>
#include <templ/solvers/GQReasoner.hpp>

using namespace templ;
using namespace templ::solvers::temporal;
using namespace templ::solvers::temporal::point_algebra;

typedef point_algebra::QualitativeTimePointConstraint QTPC;



QualitativeTemporalConstraintNetwork::Ptr createTCN(size_t numberOfTimepoints)
{
    QualitativeTemporalConstraintNetwork::Ptr tcn = make_shared< QualitativeTemporalConstraintNetwork>();

    for(size_t i = 0; i < numberOfTimepoints; )
    {
        QualitativeTimePoint::Ptr t0, t1;
        {
            std::stringstream ss;
            ss << "tp" << i;
            t0 = make_shared<QualitativeTimePoint>(ss.str());
        }
        ++i;
        {
            std::stringstream ss;
            ss << "tp" << i;
            t1 = make_shared<QualitativeTimePoint>(ss.str());
        }
        ++i;

        tcn->addQualitativeConstraint(t1, t0, QTPC::Greater);
    }

    return tcn;
}


int main(int argc, char** argv)
{
    std::map< std::string, numeric::Stats<double> > stats;
    stats["gecode"];
    stats["gq_with_overhead"];
    stats["gq"];
    stats["incremental"];

    std::cout << "# <number-of-timepoints> ";
    for(std::pair<std::string, numeric::Stats<double> > p : stats)
    {
        std::cout << "<" << p.first << "-mean> <" << p.first << "-stdev> ";
    }
    std::cout << std::endl;

    for(int i = 10; i < 502; i+=50)
    {
        for(int epoch = 0; epoch < 20; ++epoch)
        {
            QualitativeTemporalConstraintNetwork::Ptr tcn = createTCN(i);

            base::Time start =base::Time::now();
            tcn->isConsistent();
            double duration = (base::Time::now() - start).toSeconds();
            stats["gecode"].update(duration);

            start = base::Time::now();
            solvers::GQReasoner::isConsistent(*tcn);
            double gqWithOverheadDuration = (base::Time::now() - start).toSeconds();
            stats["gq_with_overhead"].update(gqWithOverheadDuration);

            solvers::GQReasoner paReasoner("point", tcn->getGraph(), make_shared<solvers::temporal::point_algebra::QualitativeTimePointConstraint>());

            start = base::Time::now();
            paReasoner.getPrimarySolution();
            double gqDuration = (base::Time::now() - start).toSeconds();
            stats["gq"].update(gqDuration);

            start = base::Time::now();
            tcn->isConsistent(QualitativeTemporalConstraintNetwork::TCN_INCREMENTAL);
            double incremental = (base::Time::now() - start).toSeconds();
            stats["incremental"].update(incremental);

            std::cout << i << " " << duration << " " << gqWithOverheadDuration << " " << gqDuration << " " << incremental << std::endl;
        }
        //std::cout << i <<  " ";
        //for(std::pair<std::string, numeric::Stats<double> > p: stats)
        //{
        //    const numeric::Stats<double>& stat = p.second;
        //    std::cout << stat.mean() << " " << stat.stdev() << " ";
        //}
        std::cout << std::endl;
    }
}

