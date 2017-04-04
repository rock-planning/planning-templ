#include <boost/test/unit_test.hpp>
#include "../../src/SpaceTime.hpp"
#include "../../src/solvers/csp/Types.hpp"
#include "../../src/solvers/csp/propagators/IsPath.hpp"
#include "../../src/solvers/csp/propagators/IsValidTransportEdge.cpp"
#include "../../src/solvers/csp/utils/Formatter.hpp"
#include <gecode/search.hh>
#include <gecode/minimodel.hh>

using namespace templ::solvers::csp;

class TestValidTransportEdge : public Gecode::Space
{
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;

    uint32_t mNumberOfVertices;
    uint32_t mNumberOfTimelines;

    std::vector<int> mSupplyDemand;
    templ::solvers::csp::ListOfAdjacencyLists mTimelines;

public:
    TestValidTransportEdge(size_t numberOfTimepoints, size_t numberOfFluents, const std::vector<int>& supplyDemand)
        : Gecode::Space()
        , mNumberOfTimepoints(numberOfTimepoints)
        , mNumberOfFluents(numberOfFluents)
        , mNumberOfVertices(numberOfTimepoints*numberOfFluents)
        , mNumberOfTimelines(supplyDemand.size())
        , mSupplyDemand(supplyDemand)
    {
        for(size_t t = 0; t < mNumberOfTimelines; ++t)
        {
            Gecode::SetVarArray timeline(*this, mNumberOfVertices, Gecode::IntSet::empty, Gecode::IntSet(0,mNumberOfVertices-1),0,1);
            mTimelines.push_back(timeline);

            // get full path length
            templ::solvers::csp::propagators::isPath(*this, timeline, numberOfTimepoints, numberOfFluents, numberOfTimepoints-1);
        }

        for(size_t t = 0; t < mNumberOfVertices; ++t)
        {
            Gecode::SetVarArray multiEdge(*this, mSupplyDemand.size());
            for(size_t i = 0; i < mSupplyDemand.size(); ++i)
            {
                multiEdge[i] = mTimelines[i][t];
            }
            propagators::isValidTransportEdge(*this, multiEdge, supplyDemand, t/mNumberOfFluents, t%mNumberOfFluents, mNumberOfFluents);

            Gecode::SetAFC afc(*this, multiEdge, 0.99);
            afc.decay(*this, 0.95);
            branch(*this, multiEdge, Gecode::SET_VAR_AFC_MIN(afc), Gecode::SET_VAL_MIN_INC());
        }

        //for(size_t t = 0; t < mNumberOfTimelines; ++t)
        //{
        //    Gecode::SetAFC afc(*this, mTimelines[t], 0.99);
        //    afc.decay(*this, 0.95);
        //    branch(*this, mTimelines[t], Gecode::SET_VAR_AFC_MIN(afc), Gecode::SET_VAL_MIN_INC());
        //}

    }

    TestValidTransportEdge(bool share, TestValidTransportEdge& other)
        : Gecode::Space(share, other)
        , mNumberOfVertices(other.mNumberOfVertices)
        , mNumberOfFluents(other.mNumberOfFluents)
        , mNumberOfTimepoints(other.mNumberOfTimepoints)
        , mNumberOfTimelines(other.mNumberOfTimelines)
        , mSupplyDemand(other.mSupplyDemand)
    {
        for(size_t i = 0; i < other.mTimelines.size(); ++i)
        {
            AdjacencyList array;
            mTimelines.push_back(array);
            mTimelines[i].update(*this, share, other.mTimelines[i]);
        }
    }

    virtual Gecode::Space* copy(bool share)
    {
        return new TestValidTransportEdge(share, *this);
    }

    std::string toString() const
    {
        std::stringstream ss;
        ss << "Solution (" << mNumberOfTimelines << ")" << std::endl;
        for(size_t i = 0; i < mNumberOfTimelines; ++i)
        {
            ss << "Timeline (" << i << ")" << std::endl;
            ss << utils::Formatter::toString(mTimelines[i], mNumberOfFluents);
            ss << std::endl;
        }
        return ss.str();
    }
};

void run_test(uint32_t numberOfTimepoints, uint32_t numberOfFluents, std::vector<int> supplyDemand, const std::string& tag)
{
    TestValidTransportEdge* testSpace = new TestValidTransportEdge(numberOfTimepoints, numberOfFluents, supplyDemand);
    Gecode::BAB<TestValidTransportEdge> searchEngine(testSpace);
    TestValidTransportEdge* best = NULL;
    TestValidTransportEdge* current = NULL;
    bool foundSolution = false;

    BOOST_TEST_MESSAGE("Run search");
    while((current = searchEngine.next()))
    {
        delete best;
        best = current;

        try {
            BOOST_REQUIRE_MESSAGE(true, "Found solution for " << tag << " -- " << best->toString());
            break;
        } catch(const std::exception& e)
        {
            BOOST_REQUIRE_MESSAGE(false, "\n" << best->toString() << "\n" << tag << " Solution not valid: " << e.what());
        }
    }
    if(!current)
    {
        BOOST_REQUIRE_MESSAGE(false, "\n No solution found for " << tag );
    }
    if(best == NULL)
    {
        delete current;
    }

    std::cout << "Statistics: " << std::endl;
    std::cout << "    propagate " << searchEngine.statistics().propagate << std::endl;

    std::cout << "    fail " << searchEngine.statistics().fail << std::endl;
    std::cout << "    node " << searchEngine.statistics().node << std::endl;
    std::cout << "    depth" << searchEngine.statistics().depth << std::endl;
    std::cout << "    restart " << searchEngine.statistics().restart << std::endl;
    std::cout << "    nogoods " << searchEngine.statistics().nogood << std::endl;

}

BOOST_AUTO_TEST_SUITE(propagators)

BOOST_AUTO_TEST_CASE(is_valid_timeline)
{
    std::vector<int> supplyDemand;
    for(int i = 0; i < 2; ++i)
    {
        supplyDemand.push_back(10);
        supplyDemand.push_back(-1);
        supplyDemand.push_back(-1);
    }

    run_test(5,5, supplyDemand, "5x5");
}

BOOST_AUTO_TEST_CASE(is_valid_timeline_25_x_25)
{
    std::vector<int> supplyDemand;
    for(int i = 0; i < 2; ++i)
    {
        supplyDemand.push_back(10);
        supplyDemand.push_back(-1);
        supplyDemand.push_back(-1);
    }

    run_test(25,25, supplyDemand, "5x5");
}

BOOST_AUTO_TEST_CASE(is_valid_timeline_50_x_50)
{
    std::vector<int> supplyDemand;
    for(int i = 0; i < 2; ++i)
    {
        supplyDemand.push_back(10);
        supplyDemand.push_back(-1);
        supplyDemand.push_back(-1);
    }

    run_test(25,25, supplyDemand, "5x5");
}

BOOST_AUTO_TEST_CASE(is_valid_timeline_50_x_50_x_5)
{
    std::vector<int> supplyDemand;
    for(int i = 0; i < 5; ++i)
    {
        supplyDemand.push_back(10);
        supplyDemand.push_back(-1);
        supplyDemand.push_back(-1);
    }

    run_test(25,25, supplyDemand, "5x5");
}

BOOST_AUTO_TEST_SUITE_END()
