#include "FlawResolution.hpp"
#include "TransportNetwork.hpp"
#include "../../SharedPtr.hpp"
#include "MissionConstraints.hpp"
#include "MissionConstraintManager.hpp"
#include <organization_model/PropertyConstraint.hpp>
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

void FlawResolution::applyResolutionOption(Gecode::Space& space, const Gecode::Space& lastSolution, const ResolutionOption& resolutionOption)
{
    const transshipment::Flaw& flaw = resolutionOption.first;
    size_t alternative = resolutionOption.second;

    TransportNetwork& currentSpace = static_cast<TransportNetwork&>(space);
    const TransportNetwork& lastSpace = static_cast<const TransportNetwork&>(lastSolution);

    ga::ConstraintViolation::Type violationType = flaw.getViolation().getType();
    FluentTimeResource::List ftrs = getAffectedRequirements(flaw.getSpaceTime(),
            violationType, lastSpace.mResourceRequirements);

    std::cout << "Apply resolution option for: " << flaw.toString(4) << std::endl;
    switch(violationType)
    {
        //case ga::ConstraintViolation::TransFlow:
            //std::cout << "Transflow violation: resolver option #" << alternative << std::endl;
            //{
            //    switch(alternative)
            //    {
            //        case 0:
            //        {
            //            //std::cout
            //            //    << "    adding distiction constraint" << std::endl
            //            //    << "         distinction for role " << flaw.affectedRole().getModel() << std::endl
            //            //    << "         current space " << &currentSpace << std::endl
            //            //    << "         last space " << &lastSpace << std::endl
            //            //    << std::endl;

            //            std::set<Role> uniqueRoles = MissionConstraints::getUniqueRoles(lastSpace.mRoleUsage,
            //                    currentSpace.mRoles,
            //                    currentSpace.mResourceRequirements,
            //                    ftrs,
            //                    flaw.affectedRole().getModel());

            //            constraints::ModelConstraint::Ptr constraint = make_shared<constraints::ModelConstraint>(constraints::ModelConstraint::MIN_DISTINCT,
            //                    flaw.affectedRole().getModel(),
            //                    MissionConstraintManager::mapToSpaceTime(ftrs),
            //                    uniqueRoles.size() + 1);

            //            currentSpace.addConstraint( constraint );
            //            break;
            //        }
            //        default:
            //            break;
            //    }
            //    break;
            //}

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

                            std::cout
                                << "    adding distiction constraint" << std::endl
                                << "        additional distinction for: " << abs(flaw.getViolation().getDelta()) << " and role: " << flaw.affectedRole().toString() << std::endl
                                << "        current space " << &currentSpace << std::endl
                                << "        last space " << &lastSpace << std::endl
                                << constraint->toString(8);


                            currentSpace.addConstraint( constraint );

                            break;
                    }
                    case 1:
                    {
                        std::cout << "    add model requirement: for min one transport provider" << std::endl;
                        std::cout << "        at spacetime: " << std::endl
                                << SpaceTime::toString(flaw.getSpaceTime(), 8);
                        std::cout << "        Resulting requirements: " << std::endl;

                        if(TransportNetwork::msInteractive)
                        {
                            std::cout << "Add function requirement: available resources are" << std::endl;
                            std::cout << currentSpace.mResources << std::endl;
                            std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
                        }


                        FluentTimeResource::List ftrs = getAffectedRequirements(flaw.getSpaceTime(),
                            violationType, lastSpace.mResourceRequirements, true);

                        constraints::ModelConstraint::Ptr constraint = make_shared<constraints::ModelConstraint>(
                                constraints::ModelConstraint::MIN_FUNCTION,
                                organization_model::vocabulary::OM::resolve("TransportProvider"),
                                MissionConstraintManager::mapToSpaceTime(ftrs),
                                1
                                );

                        currentSpace.addConstraint( constraint );
                        break;
                    }
                    default:
                        break;
                }
            break;
            }
        case ga::ConstraintViolation::TransFlow:
            std::cout << "Transflow violation: resolver option #" << alternative << std::endl;
            break;
        case ga::ConstraintViolation::TotalTransFlow:
            switch(alternative)
            {
                case 0:
                    std::cout << "    add model requirement: for transport provider" << std::endl;
                    std::cout << "        for min items: " << flaw.getViolation().getDelta() << std::endl;
                    std::cout << "        at spacetime: " << std::endl
                            << SpaceTime::toString(flaw.getSpaceTime(),8);

                    if(TransportNetwork::msInteractive)
                    {
                        std::cout << "Add function requirement: available resources are" << std::endl;
                        std::cout << currentSpace.mResources << std::endl;
                        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
                    }

                    FluentTimeResource::List ftrs = getAffectedRequirements(flaw.getSpaceTime(),
                        violationType, lastSpace.mResourceRequirements, true);

                    constraints::ModelConstraint::Ptr constraint = make_shared<constraints::ModelConstraint>(
                            constraints::ModelConstraint::MIN_PROPERTY,
                            organization_model::vocabulary::OM::resolve("TransportProvider"),
                            MissionConstraintManager::mapToSpaceTime(ftrs),
                            flaw.getViolation().getInFlow() + abs(flaw.getViolation().getDelta()),
                            organization_model::vocabulary::OM::resolve("transportCapacity")
                            );

                    currentSpace.addConstraint(constraint);
                break;
            }
            break;
        case ga::ConstraintViolation::TotalMinFlow:
            switch(alternative)
            {
            }
            break;
        default:
            std::cout << "Unknown violation constraint while try resolution" << std::endl;
            break;
    }
}


std::vector< std::pair<FlawResolution::ResolutionOption, uint32_t> > FlawResolution::selectBestResolution(Gecode::Space& space, const Gecode::Space& lastSolution, uint32_t existingCost, const FlawResolution::ResolutionOptions& resolutionOptions)
{
    Gecode::Space* master = space.clone();

    if(TransportNetwork::msInteractive)
    {
        std::cout << "FlawResolution: try finding best resolver: resolution options size " << resolutionOptions.size() << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }

    EvaluationList improvingFlaws;

    for(ResolutionOption option : resolutionOptions)
    {
        if(TransportNetwork::msInteractive)
        {
            std::cout << "FlawResolution: try resolution options, existing cost: " << existingCost << std::endl;
            std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }

        Gecode::Space* slave = master->clone();
        TransportNetwork* transportNetwork = dynamic_cast<TransportNetwork*>(slave);

        FlawResolution::applyResolutionOption(*transportNetwork, lastSolution, option);

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
        if(solution)
        {
            std::cout << "FlawResolution: found improving flaw resolver" << std::endl;
            std::cout << "    cost: " << solution->cost() << std::endl;
            std::cout << "    existing cost: " << existingCost << std::endl;
            std::cout << "    remaining flaws: " << solution->mNumberOfFlaws.val() << std::endl;
            if(TransportNetwork::msInteractive)
            {
                std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
            }

            uint32_t delta = existingCost - (uint32_t) solution->cost().val();
            Evaluation evaluation(option, delta);
            improvingFlaws.push_back(evaluation);
            if(solution->mNumberOfFlaws.val() == 0)
            {
                improvingFlaws.clear();
                improvingFlaws.push_back(evaluation);
                // break when zero flaw solution has been found
                break;
            }
        } else {
            std::cout << "FlawResolution: no improving flaw resolver (no solution found with constraint included)" << std::endl;
            std::cout << "    cost: n/a" << std::endl;
            std::cout << "    existing cost: " << existingCost << std::endl;
            if(TransportNetwork::msInteractive)
            {
                std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
            }
        }
        delete solution;
    }
    delete master;
    if(TransportNetwork::msInteractive)
    {
        std::cout << "FlawResolution: done finding best resolver" << std::endl;
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }

    return improvingFlaws;
}

FluentTimeResource::List FlawResolution::getAffectedRequirements(const SpaceTime::Point& spacetime,
            graph_analysis::algorithms::ConstraintViolation::Type violationType, const FluentTimeResource::List allRequirements, bool single)
{
    FluentTimeResource::List affected;
    for(const FluentTimeResource& ftr : allRequirements)
    {
        if(ftr.getLocation() == spacetime.first && (ftr.getInterval().getFrom() == spacetime.second ||
                ftr.getInterval().getTo() == spacetime.second))
        {
            affected.push_back(ftr);
        }
    }
    if(single)
    {
        return affected;
    }

    switch(violationType)
    {
        case ga::ConstraintViolation::MinFlow:
        case ga::ConstraintViolation::TotalMinFlow:
            break;
        case ga::ConstraintViolation::TransFlow:
        case ga::ConstraintViolation::TotalTransFlow:
            break;
        default:
            break;
    }
    return affected;
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
