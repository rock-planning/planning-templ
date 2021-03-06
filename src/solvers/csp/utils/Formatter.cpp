#include "Formatter.hpp"
#include "FluentTimeIndex.hpp"
#include <sstream>
#include <iomanip>

namespace templ {
namespace solvers {
namespace csp {
namespace utils {

std::string Formatter::toString(const std::vector<Gecode::IntVarArray>& arrays, const std::vector<Symbol::Ptr>& fluents, const std::vector<Variable::Ptr>& variables, const std::vector<std::string>& arrayLabels)
{
    std::stringstream ss;
    for(size_t i = 0; i < arrays.size(); ++i)
    {
        //assert( activeRoles.size() > i);
        ss << "Label: '" << arrayLabels[i] << "'" << std::endl;

        ss << toString(arrays[i], fluents, variables) << std::endl;
    }
    return ss.str();
}

std::string Formatter::toString(const Gecode::IntVarArray& array, const std::vector<Symbol::Ptr>& fluents, const std::vector<Variable::Ptr>& variables)
{
    size_t fluentCount = fluents.size();
    size_t variablesCount = variables.size();
    size_t columnSize = fluentCount*variablesCount;
    Gecode::Matrix<Gecode::IntVarArray> timeline(array, columnSize, columnSize);

    std::stringstream ss;
    std::string path;

    for(size_t row = 0; row < columnSize; ++row)
    {
        FluentTimeIndex fromIdx = FluentTimeIndex::fromRowOrCol(row, fluentCount, variablesCount );
        size_t fromFluentIdx = fromIdx.getFluentIndex();
        size_t fromTimeIdx = fromIdx.getTimeIndex();

        std::string fromLabel = "" + variables[fromTimeIdx]->toString() + "-" + fluents[fromFluentIdx]->toString();
        ss << "#" << std::setw(3) << row << " " << std::setw(65) << fromLabel << " -- ";
        for(size_t col = 0; col < columnSize; ++col)
        {
            FluentTimeIndex toIdx = FluentTimeIndex::fromRowOrCol(col, fluentCount, variablesCount);
            size_t toFluentIdx = toIdx.getFluentIndex();
            size_t toTimeIdx = toIdx.getTimeIndex();

            Gecode::IntVar var = timeline(col,row);
            ss <<  var << " ";
            if(var.assigned())
            {
                std::string toLabel = "" + variables[toTimeIdx]->toString() + "-" + fluents[toFluentIdx]->toString();

                Gecode::IntVarValues v(var);
                if(v.val() == 1)
                {
                    path += "from: " + fromLabel  + " to: " + toLabel + "\n";
                }
            }
        }
        ss << std::endl;
    }
    ss << std::endl << path << std::endl;
    return ss.str();
}

std::string Formatter::toString(const Gecode::ViewArray<Gecode::Set::SetView>& array,
    size_t columnSize)
{
    std::stringstream ss;
    ss << std::string(12,' ');
    for(int i = 0; i < array.size(); ++i)
    {
        if(i%columnSize == 0)
        {
            ss << std::endl;
        }
        ss << std::setw(24) << array[i];
    }
    return ss.str();
}

std::string Formatter::toString(const Gecode::SetVarArray& array, size_t columnSize)
{
    std::stringstream ss;
    for(int i = 0; i < array.size(); ++i)
    {
        if(i%columnSize == 0)
        {
            ss << std::endl;
        }
        ss << std::setw(25) << array[i];
    }
    return ss.str();
}

static std::string toString(const Gecode::ViewArray<Gecode::Set::SetView>& array,
        const symbols::constants::Location::PtrList locations,
        const temporal::point_algebra::TimePoint::PtrList& timepoints)
{
    size_t columnSize = locations.size();

    std::stringstream ss;
    ss << std::string(11,' ');
    for(const symbols::constants::Location::Ptr& location : locations)
    {
        ss << std::setw(25) << location->getInstanceName();
    }
    for(int i = 0; i < array.size(); ++i)
    {
        size_t fluentIdx = i%columnSize;
        if(fluentIdx == 0)
        {
            ss << std::endl;
            ss << std::setw(12) << timepoints[i]->toString();
        }
        ss << std::setw(24) << array[i];
    }
    return ss.str();
}

std::string Formatter::toString(const std::vector<Gecode::SetVarArray>& array,
    const symbols::constants::Location::PtrList& locations,
    const temporal::point_algebra::TimePoint::PtrList& timepoints)
{
    std::stringstream ss;
    std::vector<Gecode::SetVarArray>::const_iterator cit = array.begin();
    for(; cit != array.end(); ++cit)
    {
        ss << toString(*cit, locations, timepoints);
        ss << std::endl;
    }
    return ss.str();
}

std::string Formatter::toString(const Gecode::SetVarArray& array,
    const symbols::constants::Location::PtrList& locations,
    const temporal::point_algebra::TimePoint::PtrList& timepoints)
{
    size_t columnSize = locations.size();

    std::stringstream ss;
    ss << std::string(11,' ');
    for(const symbols::constants::Location::Ptr& location : locations)
    {
        ss << std::setw(25) << location->getInstanceName();
    }
    size_t timepointIdx = 0;
    for(int i = 0; i < array.size(); ++i)
    {
        size_t locationIdx = i%columnSize;
        if(locationIdx == 0)
        {
            ss << std::endl;
            ss << std::setw(12) << timepoints[timepointIdx++]->getLabel();
        }
        ss << std::setw(24) << array[i];
    }
    return ss.str();
}


std::string Formatter::toString(const Gecode::IntVarArray& roleUsage, const Role::List& roles, const FluentTimeResource::List& requirements)
{
    size_t firstcolumnwidth = 30;
    size_t columnwidth = 20;
    std::stringstream ss;
    ss << "Role usage:" << std::endl;
    ss << std::setw(firstcolumnwidth) << std::right << "    FluentTimeResource: ";
    for(const FluentTimeResource& fts : requirements)
    {
        /// construct string for proper alignment
        std::string s = fts.getFluent()->getInstanceName();
        s += "@[" + fts.getInterval().toString(0,true) + "]";

        ss << std::setw(columnwidth) << std::left << s;
    }
    ss << std::endl;

    for(size_t i = 0; i < roles.size(); ++i)
    {
        ss << std::setw(firstcolumnwidth) << std::left << roles[i].getName() << ": ";
        for(size_t r = 0; r < requirements.size(); ++r)
        {
            ss << std::setw(columnwidth) << roleUsage[r*roles.size() + i] << " ";
        }
        ss << std::endl;
    }
    return ss.str();
}

} // end namespace utils
} // end namespace csp
} // end namespace solvers
} // end namespace templ
