#ifndef TEMPL_TIMEPOINT_HPP
#define TEMPL_TIMEPOINT_HPP

#include <string>
#include <vector>

namespace templ {

/**
 * \class Timepoint
 * \brief Simple timepoint variable
 */
class Timepoint 
{
public:
    typedef std::string Label;
    typedef std::vector<Label> LabelList;

    Timepoint(const std::string& label)
        : mLabel(label)
    {}

private:
    Label mLabel;
};

} // end namespace templ
#endif // TEMPL_TIMEPOINT_HPP
