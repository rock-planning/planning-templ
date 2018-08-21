#ifndef TEMPL_SOLVERS_CSP_CONSTRAINT_MATRIX_HPP
#define TEMPL_SOLVERS_CSP_CONSTRAINT_MATRIX_HPP

#include <string>
#include <owlapi/model/IRI.hpp>

namespace templ {
namespace solvers {
namespace csp {

/**
 * \class ConstraintMatrix
 * \brief A matrix that stores min/max values
 * \details For debugging purposes due to the use of std::map the matrix offers lazy initialization
 */
class ConstraintMatrix
{
    struct MinMax
    {
        uint32_t min;
        uint32_t max;

        std::string toString() const;
    };

    typedef uint32_t RowId, ColumnId;
    typedef MinMax Column;

    // Requirement mapped to the min,max setting
    std::map<RowId, std::map<ColumnId, MinMax> > mMatrix;
    owlapi::model::IRIList mAvailableModels;
public:

    /**
     * Default constructor of the constraint matrix
     */
    ConstraintMatrix(const owlapi::model::IRIList& availableModels);

    void setMax(RowId row, ColumnId col, uint32_t max) {  mMatrix[row][col].max = max; }
    void setMin(RowId row, ColumnId col, uint32_t min) {  mMatrix[row][col].min = min; }

    std::string toString(const std::vector<std::string>& row_name = std::vector<std::string>()) const;
};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_CONSTRAINT_MATRIX_HPP
