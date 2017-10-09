#include "FlawResolution.hpp"

namespace templ {
namespace solvers {
namespace csp {

FlawResolution::FlawResolution()
    : mGenerator(std::random_device()())
{
}

FlawResolution::FlawResolution(const FlawResolution& other)
    : mGenerator(other.mGenerator)
    , mDraws(other.mDraws)
    , mCurrentDraw(other.mCurrentDraw)
    , mResolutionOptions(other.mResolutionOptions)
{
}

void FlawResolution::prepare(const std::vector<transshipment::Flaw>& flaws)
{
    namespace ga = graph_analysis::algorithms;
    mResolutionOptions.clear();

    if(flaws.empty())
    {
        return;
    }

    // Add the flaw with the number of resolution option
    for(const transshipment::Flaw& flaw : flaws)
    {
        switch(flaw.violation.getType())
        {
            case ga::ConstraintViolation::TransFlow:
                mResolutionOptions.push_back( ResolutionOption(flaw,0) );
                break;
            case ga::ConstraintViolation::MinFlow:
                mResolutionOptions.push_back( ResolutionOption(flaw,0) );
                mResolutionOptions.push_back( ResolutionOption(flaw,1) );
                break;
            default:
                LOG_WARN_S << "Unknown flaw type";
                break;
        }
    }

    Draw indices;
    for(size_t i = 0; i < mResolutionOptions.size(); ++i)
    {
        indices.push_back(i);
    }

    numeric::Combination<size_t> combinations(indices, indices.size(), numeric::MAX);
    do {
        Draw combination = combinations.current();
        mDraws.push_back(combination);
    } while(combinations.next());
}

FlawResolution::ResolutionOptions FlawResolution::current() const
{
    return select<ResolutionOption>(mResolutionOptions, mCurrentDraw);
}

std::string FlawResolution::toString(const ResolutionOptions& options)
{
    std::stringstream ss;
    ss << "Combinations:" << std::endl;
    ss << "[ ";
    for(const ResolutionOption& option : options)
    {
        const transshipment::Flaw& flaw = option.first;
        ss << flaw.toString() + " -- alternative " << option.second << " ";
    }
    ss << "]";
    return ss.str();
}

std::string FlawResolution::toString(const Draw& draw)
{
    std::stringstream ss;
    ss << "Combinations:" << std::endl;
    ss << "[ ";
    for(size_t idx : draw)
    {
        ss << idx << " ";
    }
    ss << "]";
    return ss.str();
}

bool FlawResolution::next(bool random) const
{
    if(mDraws.empty())
    {
        return false;
    }

    if(!random)
    {
        mCurrentDraw = mDraws.back();
        mDraws.pop_back();
    } else {
        size_t optionsSize = mDraws.size();
        std::uniform_int_distribution<> dis(0,optionsSize-1);
        size_t idx = dis(mGenerator);
        mCurrentDraw = mDraws.at(idx);
        mDraws.erase( mDraws.begin() + idx);
        mDraws.resize(optionsSize-1);
    }
    return true;
}



} // end namespace csp
} // end namespace solvers
} // end namespace templ
