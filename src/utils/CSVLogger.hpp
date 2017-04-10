#ifndef TEMPL_UTILS_CSV_LOGGER_HPP
#define TEMPL_UTILS_CSV_LOGGER_HPP

#include "Logger.hpp"

namespace templ {

class CSVLogger
{
public:
    typedef std::vector<double> Row;
    typedef std::vector<std::string> ColumnDescription;

    CSVLogger();
    CSVLogger(const ColumnDescription& columns);

    void setColumnDescription(const ColumnDescription& d) { mColumnDescription = d; }
    const ColumnDescription& getColumnDescription() const { return mColumnDescription; }

    void prepareRow();
    void addToRow(double d, size_t index);
    void addToRow(double d, const std::string& description);
    void appendToRow(double d);
    void commitRow();

    void save(const std::string& filename, bool withColumnDescription = true) const;

    size_t getNumberOfColums() const { return mColumnDescription.size(); }
    size_t getNumberOfRows() const { return mRows.size(); }

    double get(size_t rowIdx, size_t colIdx) { return mRows.at(rowIdx).at(colIdx); }


private:
    ColumnDescription mColumnDescription;
    std::vector< Row > mRows;
    Row mCurrentRow;
    size_t mCurrentRowIndex;
};

} // end namespace templ
#endif // TEMPL_UTILS_CSV_LOGGER_HPP
