#include "Flaw.hpp"

using namespace graph_analysis::algorithms;

namespace templ {
namespace solvers {
namespace transshipment {

std::string Flaw::toString(size_t indent) const
{
    std::stringstream ss;
    std::string hspace(indent,' ');
    switch(violation.getType())
    {
        case ConstraintViolation::MinFlow:
                ss << hspace << "Minflow violation in timeline:" << std::endl
                    << hspace << "    enforce distiction on timeline for: " << affectedRole.toString() << std::endl
                    << hspace << " between " << std::endl
                    << hspace << previousFtr.toString() << " and " << ftr.toString() << std::endl
                    << hspace << previousFtr.getInterval().toString() << std::endl
                    << hspace << ftr.getInterval().toString() << std::endl
                    ;
                    //<< " timeline: " << std::endl
                    //<< roleTimeline.toString()
                    //<< std::endl;
                break;
        case ConstraintViolation::TransFlow:
                ss << hspace << "Transflow violation in timeline:" << std::endl
                    << hspace << "    enforce distiction on timeline for: " << affectedRole.toString() << std::endl
                    << hspace << " between " << std::endl
                    << hspace << ftr.toString() << " and " << subsequentFtr.toString() << std::endl
                    << hspace << ftr.getInterval().toString() << std::endl
                    << hspace << subsequentFtr.getInterval().toString() << std::endl
                    ;
                    //<< " timeline: " << std::endl
                    //<< roleTimeline.toString()
                    //<< std::endl;
                break;

    }
    return ss.str();
}

} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
