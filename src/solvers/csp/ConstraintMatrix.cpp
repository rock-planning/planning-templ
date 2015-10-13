#include "ConstraintMatrix.hpp"

#include <sstream>
#include <iomanip>

namespace templ {
namespace solvers {
namespace csp {

std::string ConstraintMatrix::MinMax::toString() const
{
    std::stringstream ss;
    ss << std::setw(4) << "(" << min << "," << max << ")";
    return ss.str();
}

ConstraintMatrix::ConstraintMatrix(const owlapi::model::IRIList& availableModels)
    : mAvailableModels(availableModels)
{}

std::string ConstraintMatrix::toString() const 
{
    std::stringstream ss;
    ss << "Constraint matrix:" << std::endl;
    ss << "Available models: " << mAvailableModels << std::endl;
    for(auto row : mMatrix)
    {
        ss << "#" << std::setw(4) << row.first << " ";
        
        for(auto col : row.second)
        {
            ss << " " << col.second.toString();
        }
        ss << std::endl;
    }
    return ss.str();
}


} // end namespace csp
} // end namespace solvers
} // end namespace templ
