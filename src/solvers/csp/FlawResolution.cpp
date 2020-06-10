#include "FlawResolution.hpp"
#include "TransportNetwork.hpp"
#include "../../SharedPtr.hpp"
#include "MissionConstraints.hpp"
#include "MissionConstraintManager.hpp"
#include <moreorg/PropertyConstraint.hpp>
#include "../../constraints/ModelConstraint.hpp"

namespace ga = graph_analysis::algorithms;

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
    mResolutionOptions.clear();
    mDraws.clear();

    if(flaws.empty())
    {
        return;
    }

    // Add the flaw with the number of resolution option
    for(const transshipment::Flaw& flaw : flaws)
    {
        switch(flaw.getViolation().getType())
        {
            case ga::ConstraintViolation::TransFlow:
            case ga::ConstraintViolation::TotalTransFlow:
                mResolutionOptions.push_back(ResolutionOption(flaw,0) );
                break;
            case ga::ConstraintViolation::MinFlow:
            case ga::ConstraintViolation::TotalMinFlow:
                //mResolutionOptions.push_back(ResolutionOption(flaw,0) );
                //mResolutionOptions.push_back( ResolutionOption(flaw,1) );
                break;
            case ga::ConstraintViolation::FlowBalance:
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

    if(!indices.empty())
    {
        numeric::Combination<size_t> combinations(indices, 1, numeric::MAX);
        do {
            Draw combination = combinations.current();
            mDraws.push_back(combination);
        } while(combinations.next());
    }
}

FlawResolution::ResolutionOptions FlawResolution::current() const
{
    if(TransportNetwork::msInteractive)
    {
        std::cout << "Resolutions options: " << mResolutionOptions.size() << " draw:" << toString(mCurrentDraw);
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }
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

Constraint::PtrList FlawResolution::selectBestResolution(Gecode::Space& space,
        const Gecode::Space& lastSolution,
        uint32_t existingCost,
        const FlawResolution::ResolutionOptions& resolutionOptions)
{
    Constraint::PtrList constraints = translate(space, lastSolution, resolutionOptions);
    if(constraints.empty())
    {
        throw std::invalid_argument("templ::solvers::csp::FlawResolution::selectBestResolution: "
                " no constraints given");
    }

    const TransportNetwork& transportNetwork = dynamic_cast<const TransportNetwork&>(lastSolution);

    double cost = transportNetwork.cost().val();
    double improvedCost = cost;

    EvaluationList evaluationList;
    Constraint::PtrList testGroup;
    for(size_t a = 0; a < constraints.size(); ++a)
    {
        testGroup.push_back( constraints[a] );

        Evaluation eval = evaluate(space, lastSolution, testGroup);
        if(eval.second >= improvedCost)
        {
            // remove constraint that deteriorated/did not improve the solution
            testGroup.pop_back();
            continue;
        } else {
            evaluationList.push_back(eval);
            improvedCost = eval.second;
        }

        if(improvedCost == 0)
        {
            return testGroup;
        }
    }

    std::sort(evaluationList.begin(), evaluationList.end(), [](const Evaluation& a, const Evaluation& b)
            {
                return a.second < b.second;
            });

    if(evaluationList.empty())
    {
        return Constraint::PtrList();
    }

    return evaluationList.front().first;
}

FlawResolution::Evaluation FlawResolution::evaluate(Gecode::Space& space,
        const Gecode::Space& lastSolution,
        const Constraint::PtrList& constraints)
{
    Gecode::Space* master = space.clone();
    TransportNetwork* transportNetwork = dynamic_cast<TransportNetwork*>(master);
    MissionConstraintManager::apply(constraints, *transportNetwork);

    Gecode::Search::Options options;
    options.threads = 1;
    Gecode::Search::Cutoff * c = Gecode::Search::Cutoff::constant(1);
    options.cutoff = c;
    // p 172 "the value of nogoods_limit described to which depth limit
    // no-goods should be extracted from the path of the search tree
    // maintained by the search engine
    options.nogoods_limit = 128;
    // recomputation distance
    //options.c_d = 30;
    // adaptive recomputation distance
    // options.a_d =
    // default node cutoff
    // options.node =
    // default failure cutoff
    // options.fail
    options.stop = Gecode::Search::Stop::time(30000);
    transportNetwork->setUseMasterSlave(false);
    Gecode::RBS<TransportNetwork, Gecode::DFS> searchEngine(transportNetwork, options);
    TransportNetwork* solution = searchEngine.next();
    double cost = std::numeric_limits<double>::max();

    if(solution)
    {
        cost = solution->cost().val();
        delete solution;
        delete master;
    }

    return Evaluation(constraints, cost);
}

FluentTimeResource::List FlawResolution::getAffectedRequirements(const SpaceTime::Point& spacetime,
            graph_analysis::algorithms::ConstraintViolation::Type violationType,
            const FluentTimeResource::List allRequirements)
{
    if(allRequirements.empty())
    {
        throw std::invalid_argument("templ::solvers::csp::FlawResolution::getAffectedRequirements: requirements list is empty");
    }

    FluentTimeResource::List affected;
    for(const FluentTimeResource& ftr : allRequirements)
    {
        if(ftr.getLocation()->equals(spacetime.first))
        {
            if(ftr.getInterval().contains(spacetime.second))
            {
                affected.push_back(ftr);
            }
        }
    }

    // if not affected requirement is found, then
    // the flaw has not relevant with respect to the actual requirement of the
    // mission
    // hence no additional action should be required
    if(affected.empty())
    {
        throw std::invalid_argument("templ::solvers::csp::FlawResolution::getAffectedRequirements: failed"
                " to identify requirement for spacetime -- so no relevance: " + spacetime.first->toString() + " " + spacetime.second->toString() +
                FluentTimeResource::toString(allRequirements, 8)
                );
    }

    return affected;
}

Constraint::Ptr FlawResolution::translate(Gecode::Space& space,
        const Gecode::Space& lastSolution,
        const ResolutionOption& resolutionOption)
{
    const transshipment::Flaw& flaw = resolutionOption.first;
    size_t alternative = resolutionOption.second;

    TransportNetwork& currentSpace = static_cast<TransportNetwork&>(space);
    const TransportNetwork& lastSpace = static_cast<const TransportNetwork&>(lastSolution);

    ga::ConstraintViolation::Type violationType = flaw.getViolation().getType();
    FluentTimeResource::List ftrs = getAffectedRequirements(flaw.getSpaceTime(),
            violationType, lastSpace.mResourceRequirements);

    switch(violationType)
    {
        case ga::ConstraintViolation::MinFlow:
            std::cout << "Minflow violation: resolver option #" << alternative << std::endl;
            {
                switch(alternative)
                {
                    case 0:
                    {
                            std::set<Role> uniqueRoles = MissionConstraints::getUniqueRoles(lastSpace.mRoleUsage,
                                    currentSpace.mRoles,
                                    currentSpace.mResourceRequirements,
                                    ftrs,
                                    flaw.affectedRole().getModel());

                            constraints::ModelConstraint::Ptr constraint = make_shared<constraints::ModelConstraint>(constraints::ModelConstraint::MIN_DISTINCT,
                                    flaw.affectedRole().getModel(),
                                    MissionConstraintManager::mapToSpaceTime(ftrs),
                                    uniqueRoles.size() + abs(flaw.getViolation().getDelta()) );

                            return constraint;
                    }
                    case 1:
                    {
                        FluentTimeResource::List ftrs = getAffectedRequirements(flaw.getSpaceTime(),
                            violationType, lastSpace.mResourceRequirements);

                        constraints::ModelConstraint::Ptr constraint = make_shared<constraints::ModelConstraint>(
                                constraints::ModelConstraint::MIN_FUNCTION,
                                moreorg::vocabulary::OM::resolve("TransportProvider"),
                                MissionConstraintManager::mapToSpaceTime(ftrs),
                                1
                                );
                        return constraint;

                    }
                    default:
                        break;
                }
            break;
            }
        case ga::ConstraintViolation::TransFlow:
            break;
        case ga::ConstraintViolation::TotalTransFlow:
            switch(alternative)
            {
                case 0:
                    FluentTimeResource::List ftrs = getAffectedRequirements(flaw.getSpaceTime(),
                        violationType, lastSpace.mResourceRequirements);

                    constraints::ModelConstraint::Ptr constraint = make_shared<constraints::ModelConstraint>(
                            constraints::ModelConstraint::MIN_PROPERTY,
                            moreorg::vocabulary::OM::resolve("TransportProvider"),
                            MissionConstraintManager::mapToSpaceTime(ftrs),
                            flaw.getViolation().getInFlow() + abs(flaw.getViolation().getDelta()),
                            moreorg::vocabulary::OM::resolve("transportCapacity")
                            );
                    return constraint;

            }
            break;
        case ga::ConstraintViolation::TotalMinFlow:
            break;
        default:
            std::cout << "Unknown violation constraint while try resolution" << std::endl;
            break;
    }
    throw std::invalid_argument("templ::solvers::csp::FlawResolution::translate: failed to converte flaw resolution option to constraint");
}

Constraint::PtrList FlawResolution::translate(Gecode::Space& space,
        const Gecode::Space& lastSolution,
        const ResolutionOptions& resolutionOptions)
{
    Constraint::PtrList constraints;
    for(const ResolutionOption& option : resolutionOptions)
    {
        try {
            Constraint::Ptr constraint = translate(space, lastSolution, option);
            if(constraint)
            {
                Constraint::PtrList::const_iterator cit = std::find_if(constraints.begin(), constraints.end(), [constraint](const Constraint::Ptr& c)
                        {
                            return *c.get() == *constraint.get();
                        });

                if(cit == constraints.end())
                {
                    constraints.push_back(constraint);
                }
            }
        } catch(const std::invalid_argument& e)
        {
            LOG_INFO_S << "Translation failed for irrelevant flaw: " << e.what();
        }
    }
    return constraints;
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
