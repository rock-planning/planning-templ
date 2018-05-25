#ifndef TEMPL_IO_LATEX_WRITER_HPP
#define TEMPL_IO_LATEX_WRITER_HPP

#include "../Mission.hpp"

namespace templ {
namespace io {

class LatexWriter
{
public:
    static std::string toLatex(const Mission::Ptr& mission);

    static std::string toLatex(const solvers::FluentTimeResource& ftr);

    static std::string suffixNumber(const std::string& label);


};

} // end namespace io
} // end namespace templ
#endif //
