#ifndef TEMPL_TIMEPOINT_HPP
#define TEMPL_TIMEPOINT_HPP

namespace templ {

class Timepoint 
{
public:
    bool operator==(const Timepoint& other) const { return true; }
};

} // end namespace templ
#endif // TEMPL_TIMEPOINT_HPP
