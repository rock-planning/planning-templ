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
    , mOptions(other.mOptions)
    , mDraw(other.mDraw)
{
}

void FlawResolution::prepare(size_t size)
{
    mDraw.clear();
    mOptions.clear();

    if(size == 0)
    {
        return;
    }

    std::vector<size_t> indices;
    for(size_t i = 0; i < size; ++i)
    {
        indices.push_back(i);
    }

    numeric::Combination<size_t> combinations(indices, indices.size(), numeric::MAX);
    do {
        std::vector<size_t> combination = combinations.current();
        mOptions.push_back(combination);
    } while(combinations.next());
}

std::string FlawResolution::toString(const Draw& draw)
{
    std::stringstream ss;
    ss << "Combinations:" << std::endl;
    ss << "[ ";
    for(size_t index : draw)
    {
        ss << index << " ";
    }
    ss << "]";
    return ss.str();
}

bool FlawResolution::next(bool random) const
{
    if(mOptions.empty())
    {
        return false;
    }

    if(!random)
    {
        mDraw = mOptions.back();
        mOptions.pop_back();
    } else {
        size_t optionsSize = mOptions.size();
        std::uniform_int_distribution<> dis(0,optionsSize-1);
        size_t idx = dis(mGenerator);
        mDraw = mOptions.at(idx);
        mOptions.erase( mOptions.begin() + idx);
        mOptions.resize(optionsSize-1);
    }
    return true;
}



} // end namespace csp
} // end namespace solvers
} // end namespace templ
