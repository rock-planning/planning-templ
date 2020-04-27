#include <iostream>
#include <graph_analysis/GraphIO.hpp>
#include "../solvers/temporal/TemporalConstraintNetwork.hpp"
#include "../solvers/temporal/QualitativeTemporalConstraintNetwork.hpp"

/// Dumping a temporal constraint network
int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cout << "usage: " << argv[0] << " <filename>" << std::endl;
        return 0;
    }

    using namespace templ::solvers::temporal;

    QualitativeTemporalConstraintNetwork::Ptr tpc(new QualitativeTemporalConstraintNetwork());

    using namespace graph_analysis;

    io::GraphIO::read(argv[1], tpc->getGraph());

    std::string filename = "/tmp/tpc-out.dot";
    std::cout << "Write to " << filename;
    io::GraphIO::write(filename, tpc->getGraph());
    std::cout << "Open with 'xdot " << filename << std::endl;

    return 0;
}
