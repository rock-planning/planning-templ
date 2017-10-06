#ifndef TEMPL_SOLVERS_CSP_FLAW_RESOLUTION_HPP
#define TEMPL_SOLVERS_CSP_FLAW_RESOLUTION_HPP

#include <vector>
#include <random>
#include <numeric/Combinatorics.hpp>

namespace templ {
namespace solvers {
namespace csp {

class FlawResolution
{
public:
    typedef std::vector<size_t> Draw;
    typedef std::vector< Draw > DrawList;

    FlawResolution();

    FlawResolution(const FlawResolution& other);

    /**
     * Prepare the flaw resolution for an array of the
     * given size
     */
    void prepare(size_t size);

    /**
     *
     */
    bool next(bool random = true) const;

    Draw current() const { return mDraw; }

    const DrawList& remaining() const { return mOptions; }

    bool exhausted() { return mOptions.empty(); }

    static std::string toString(const Draw& draw);

    /**
     * Select items from a list according to a given draw
     */
    template<typename T>
    static std::vector<T> select(const std::vector<T>& itemList, const Draw& draw)
    {
        std::vector<T> selection;
        for(size_t index : draw)
        {
            selection.push_back(itemList.at(index));
        }
        return selection;
    }

private:
    mutable std::mt19937 mGenerator;

    mutable DrawList mOptions;
    mutable Draw mDraw;

};
} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_FLAW_RESOLUTION_HPP
