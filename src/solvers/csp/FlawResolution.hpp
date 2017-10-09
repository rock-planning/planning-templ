#ifndef TEMPL_SOLVERS_CSP_FLAW_RESOLUTION_HPP
#define TEMPL_SOLVERS_CSP_FLAW_RESOLUTION_HPP

#include <vector>
#include <random>
#include <numeric/Combinatorics.hpp>
#include "../transshipment/Flaw.hpp"

namespace templ {
namespace solvers {
namespace csp {

class FlawResolution
{
public:

    typedef std::pair<transshipment::Flaw, size_t> ResolutionOption;
    typedef std::vector< ResolutionOption > ResolutionOptions;

    typedef std::vector<size_t> Draw;
    typedef std::vector< Draw > DrawList;

    FlawResolution();

    FlawResolution(const FlawResolution& other);

    /**
     * Prepare the flaw resolution for an array of the
     * given size
     */
    void prepare(const std::vector<transshipment::Flaw>& flaws);

    /**
     *
     */
    bool next(bool random = true) const;

    ResolutionOptions current() const;

    const DrawList& remainingDraws() const { return mDraws; }

    bool exhausted() { return mDraws.empty(); }

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

    static std::string toString(const ResolutionOptions& options);
    static std::string toString(const Draw& draw);

private:
    mutable std::mt19937 mGenerator;

    mutable DrawList mDraws;
    mutable Draw mCurrentDraw;

    mutable ResolutionOptions mResolutionOptions;

};
} // end namespace csp
} // end namespace solvers
} // end namespace templ

#endif // TEMPL_SOLVERS_CSP_FLAW_RESOLUTION_HPP
