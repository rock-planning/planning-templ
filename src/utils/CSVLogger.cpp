#include "CSVLogger.hpp"
#include <fstream>
#include <assert.h>
#include <sstream>
#include <algorithm>
#include <limits>

namespace templ {

CSVLogger::CSVLogger()
{}

CSVLogger::CSVLogger(const std::vector<std::string>& columns)
    : mColumnDescription(columns)
    , mCurrentRow(columns.size(), std::numeric_limits<double>::quiet_NaN())
    , mCurrentRowIndex(0)
{
    assert(!columns.empty());
}

void CSVLogger::prepareRow()
{
    mCurrentRow = Row(mColumnDescription.size(), std::numeric_limits<double>::quiet_NaN());
    mCurrentRowIndex = 0;
}

void CSVLogger::addToRow(double d, size_t index)
{
    mCurrentRow.at(index) = d;
}

void CSVLogger::addToRow(double d, const std::string& description)
{
    ColumnDescription::const_iterator cit = std::find(mColumnDescription.begin(), mColumnDescription.end(), description);
    if(cit == mColumnDescription.end())
    {
        std::stringstream ss;
        ss << "templ::utils::CSVLogger::addToRow could not find column description '" << description << "'";
        throw std::runtime_error(ss.str());

    }
    size_t index = cit - mColumnDescription.begin();
    addToRow(d, index);
}

void CSVLogger::appendToRow(double d)
{
    assert(mCurrentRowIndex < getNumberOfColums());
    mCurrentRow.at(mCurrentRowIndex) = d;
    ++mCurrentRowIndex;
}

void CSVLogger::commitRow()
{
    mRows.push_back( mCurrentRow );
}

void CSVLogger::save(const std::string& filename, bool withColumnDescription) const
{
    std::ofstream outfile(filename);
    if(withColumnDescription)
    {
        outfile << "# ";
        for(const std::string& description : mColumnDescription)
        {
            outfile << description << " ";
        }
        outfile << std::endl;
    }

    for(const Row& row : mRows)
    {
        for(const double& value : row)
        {
            outfile << value << " ";
        }
        outfile << std::endl;
    }
    outfile.close();
}

} // end namespace templ
