#ifndef TEMPL_BENCHMARKS_IO_GOLDER_READER_HPP
#define TEMPL_BENCHMARKS_IO_GOLDER_READER_HPP

#include <string>
#include <fstream>
#include "../VRPProblem.hpp"

namespace templ {
namespace benchmark {
namespace io {

class GoldenReader
{
public:
    VRPProblem read(const std::string& file);
    void loadNodeCoordinates(std::ifstream& infline, VRPProblem& vrp);
    void loadDepots(std::ifstream& infline, VRPProblem& vrp);
    void loadDemands(std::ifstream& infline, VRPProblem& vrp);


};

} // end namespace io
} // end namespace benchmark
} // end namespace templ
#endif // TEMPL_BENCHMARKS_IO_GOLDER_READER_HPP
