#ifndef TEMPL_TIMEPOINT_HPP
#define TEMPL_TIMEPOINT_HPP

namespace templ {

class Timepoint 
{
    std::string mLabel;
public:
    Timepoint(const std::string& label)
        : mLabel(label)
    {}

    //bool operator==(const Timepoint& other) const { return mLabel == other.mLabel; }
};

} // end namespace templ
#endif // TEMPL_TIMEPOINT_HPP
