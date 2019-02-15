#ifndef TEMPL_IO_LATEX_WRITER_HPP
#define TEMPL_IO_LATEX_WRITER_HPP

#include "../Mission.hpp"

namespace templ {
namespace io {

class LatexWriter
{
public:
    static std::string toLatex(const Mission::Ptr& mission);

    static std::string toLatex(const symbols::constants::Location::Ptr& location);

    static std::string toLatex(const solvers::FluentTimeResource& ftr);

    static std::string suffixNumber(const std::string& label);

    /**
     * Escape string and wrap in enviroment
     */
    static std::string escape(const std::string& label, const std::string& env ="\\textit");

    /**
     */
    static std::string wrap(std::ostream& os, const std::string& currentRow,
        const std::string& appendLabel,
        const std::string& suffixNoWrap = "," ,
        const std::string& suffixWrap = " \\\\\n",
        size_t lineWidth = 60);


};

} // end namespace io
} // end namespace templ
#endif //
