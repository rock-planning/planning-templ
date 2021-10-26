#ifndef TEMPL_SOLVERS_CSP_FLAW_RESOLUTION_HPP
#define TEMPL_SOLVERS_CSP_FLAW_RESOLUTION_HPP

#include <vector>
#include <random>
#include <numeric/Combinatorics.hpp>
#include "../transshipment/Flaw.hpp"
#include <gecode/search.hh>
//#include <graph_analysis/algorithms/ConstraintViolation.hpp>

namespace templ {
namespace solvers {
namespace csp {

/**
 * Class that computes the flaw resolution strategy, based
 * on a set of predefined flaw resolution options
 *
 * Computes combinations of resolution options, so that they can
 * be tested
 */
class FlawResolution
{
public:

    typedef std::pair<transshipment::Flaw, size_t> ResolutionOption;
    typedef std::vector< ResolutionOption > ResolutionOptions;

    typedef std::vector<size_t> Draw;
    typedef std::vector< Draw > DrawList;

    typedef std::pair<Constraint::PtrList, double> Evaluation;
    typedef std::vector<Evaluation> EvaluationList;

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

    static FluentTimeResource::List getAffectedRequirements(const SpaceTime::Point& timepoint,
            graph_analysis::algorithms::ConstraintViolation::Type violationType,
            const FluentTimeResource::List allRequirements);
    /**
     * Return the list of resolution options
     */
    const ResolutionOptions& getResolutionOptions() const { return mResolutionOptions; }

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

    static Constraint::PtrList selectBestResolution(Gecode::Space& space,
            const Gecode::Space& lastSolution,
            uint32_t existingCost,
            const FlawResolution::ResolutionOptions& resolutionOptions);

    static Evaluation evaluate(Gecode::Space& space,
            const Gecode::Space& lastSolution,
            const Constraint::PtrList& constraints);

    static Constraint::Ptr translate(Gecode::Space& space,
            const Gecode::Space& lastSolution,
            const ResolutionOption& resolutionOption);

    static Constraint::PtrList translate(Gecode::Space& space,
            const Gecode::Space& lastSolution,
            const ResolutionOptions& resolutionOptions);

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
