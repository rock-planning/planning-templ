#include "FlawResolution.hpp"
#include "TransportNetwork.hpp"
#include "../../SharedPtr.hpp"
#include "MissionConstraints.hpp"
#include <organization_model/PropertyConstraint.hpp>

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
    mDraws.clear();

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
            case ga::ConstraintViolation::TotalTransFlow:
                break;
            case ga::ConstraintViolation::TotalMinFlow:
                mResolutionOptions.push_back(ResolutionOption(flaw,0) );
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
    namespace ga = graph_analysis::algorithms;

    const transshipment::Flaw& flaw = resolutionOption.first;
    size_t alternative = resolutionOption.second;

    TransportNetwork& currentSpace = static_cast<TransportNetwork&>(space);
    const TransportNetwork& lastSpace = static_cast<const TransportNetwork&>(lastSolution);

    switch(flaw.violation.getType())
    {
        case ga::ConstraintViolation::TransFlow:

            std::cout << "Transflow violation: resolver option #" << alternative << std::endl;
            {
                switch(alternative)
                {
                    case 0:
                        std::cout
                            << "    adding distiction constraint" << std::endl
                            << "         distinction for role " << flaw.affectedRole().getModel() << std::endl
                            << "         current space " << &currentSpace << std::endl
                            << "         last space " << &lastSpace << std::endl
                            << std::endl;

                        MissionConstraints::addDistinct(currentSpace,
                                lastSpace.mRoleUsage,
                                currentSpace.mRoleUsage,
                                currentSpace.mRoles,
                                currentSpace.mResourceRequirements,
                                flaw.ftr,
                                flaw.subsequentFtr,
                                flaw.affectedRole().getModel(),
                                1);
                        break;
                    default:
                        break;
                }
            break;
            }

        case ga::ConstraintViolation::MinFlow:
            std::cout << "Minflow violation: resolver option #" << alternative << std::endl;
            {
                switch(alternative)
                {
                    case 0:
                        std::cout
                            << "    adding distiction constraint" << std::endl
                            << "        additional distinction for: " << abs(flaw.violation.getDelta()) << " and role: " << flaw.affectedRole().toString() << std::endl
                            << "        current space " << &currentSpace << std::endl
                            << "        last space " << &lastSpace << std::endl
                            << std::endl;

                            MissionConstraints::addDistinct(currentSpace,
                                    lastSpace.mRoleUsage,
                                    currentSpace.mRoleUsage,
                                    currentSpace.mRoles, currentSpace.mResourceRequirements,
                                    flaw.previousFtr,
                                    flaw.ftr,
                                    flaw.affectedRole().getModel(),
                                    abs( flaw.violation.getDelta() )
                                    );
                            break;
                    case 1:
                        std::cout << "    add model requirement: for min one transport provider" << std::endl;
                        std::cout << "        for fluent time resource: " << std::endl
                                << flaw.ftr.toString(8);
                        std::cout << "        Resulting requirements: " << std::endl;

                        if(TransportNetwork::msInteractive)
                        {
                            std::cout << "Add function requirement: available resources are" << std::endl;
                            std::cout << currentSpace.mResources << std::endl;
                            std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
                        }

                        MissionConstraints::addFunctionRequirement(
                                currentSpace.mResources,
                                currentSpace.mResourceRequirements,
                                flaw.ftr,
                                organization_model::vocabulary::OM::resolve("TransportProvider"),
                                currentSpace.mAsk);

                        if(TransportNetwork::msInteractive)
                        {
                            std::cout << "Fluents after adding function requirement: " << std::endl;
                            for(const FluentTimeResource& ftr : currentSpace.mResourceRequirements)
                            {
                                std::cout << ftr.toString(12) << std::endl;
                            }
                            std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
                        }
                        break;
                    default:
                        break;
                }
            break;
            }
        case ga::ConstraintViolation::TotalTransFlow:
            break;
        case ga::ConstraintViolation::TotalMinFlow:
            switch(alternative)
            {
                case 0:
                    std::cout << "    add model requirement: for transport provider" << std::endl;
                    std::cout << "        for min items: " << flaw.violation.getDelta() << std::endl;
                    std::cout << "        for fluent time resource: " << std::endl
                            << flaw.ftr.toString(8);
                    std::cout << "        Resulting requirements: " << std::endl;

                    if(TransportNetwork::msInteractive)
                    {
                        std::cout << "Add function requirement: available resources are" << std::endl;
                        std::cout << currentSpace.mResources << std::endl;
                        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
                    }

                    using namespace organization_model;
                    Functionality functionality( vocabulary::OM::resolve("TransportProvider") );
                    PropertyConstraint::Set constraints;
                    PropertyConstraint constraint( vocabulary::OM::resolve("payloadTransportCapacity"),PropertyConstraint::GREATER_EQUAL, abs(flaw.violation.getDelta()) );

                    FunctionalityRequirement functionalityRequirement(functionality, constraints);
                    FunctionalityRequirement::Map functionalityRequirements;
                    functionalityRequirements[functionality] = functionalityRequirement;

                    MissionConstraints::addFunctionalitiesRequirement(
                            currentSpace.mResources,
                            currentSpace.mResourceRequirements,
                            flaw.ftr,
                            functionalityRequirements,
                            currentSpace.mAsk);

                    if(TransportNetwork::msInteractive)
                    {
                        std::cout << "Fluents after adding function requirement: " << std::endl;
                        for(const FluentTimeResource& ftr : currentSpace.mResourceRequirements)
                        {
                            std::cout << ftr.toString(12) << std::endl;
                        }
                        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
                    }
                    break;
            }
            break;
        default:
            std::cout << "Unknown violation constraint while try resolution" << std::endl;
            break;
    }
}


std::vector< std::pair<FlawResolution::ResolutionOption, uint32_t> > FlawResolution::selectBestResolution(Gecode::Space& space, const Gecode::Space& lastSolution, uint32_t existingCost, const FlawResolution::ResolutionOptions& resolutionOptions)
{
    Gecode::Space* master = space.clone(true);

    std::cout << "FlawResolution: try finding best resolver: resolution options size " << resolutionOptions.size() << std::endl;
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

    EvaluationList improvingFlaws;

    for(ResolutionOption option : resolutionOptions)
    {
        Gecode::Space* slave = master->clone(false);
        TransportNetwork* transportNetwork = dynamic_cast<TransportNetwork*>(slave);

        FlawResolution::applyResolutionOption(*transportNetwork, lastSolution, option);

        Gecode::Search::Options options;
        options.threads = 1;
        // p 172 "the value of nogoods_limit described to which depth limit
        // no-goods should be extracted from the path of the search tree
        // maintained by the search engine
        options.nogoods_limit = 1024;
        // recomputation distance
        // options.c_d =
        // adaptive recomputation distance
        // options.a_d =
        // default node cutoff
        // options.node =
        // default failure cutoff
        // options.fail
        options.stop = Gecode::Search::Stop::time(120000);
        transportNetwork->setUseMasterSlave(false);
        Gecode::BAB<TransportNetwork> searchEngine(transportNetwork, options);
        TransportNetwork* best = NULL;
        while(TransportNetwork* current = searchEngine.next())
        {
            delete best;
            best = current;
            break;
        }
        if(best)
        {
            std::cout << "FlawResolution: found improving flaw resolver" << std::endl;
            std::cout << "    cost: " << best->cost() << std::endl;
            std::cout << "    existing cost: " << existingCost << std::endl;
            std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

            uint32_t delta = existingCost - (uint32_t) best->cost().val();
            improvingFlaws.push_back( Evaluation(option, delta) );
        }
        delete best;
    }
    delete master;
    std::cout << "FlawResolution: done finding best resolver" << std::endl;
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

    return improvingFlaws;
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
