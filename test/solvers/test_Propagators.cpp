#include <boost/test/unit_test.hpp>
#include <templ/solvers/csp/propagators/IsPath.hpp>
#include <gecode/search.hh>
#include <gecode/minimodel.hh>

class TestSpace : public Gecode::Space
{
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;

    uint32_t mNumberOfVertices;

    Gecode::IntVarArray mGraph;

public:
    TestSpace(size_t numberOfTimepoints, size_t numberOfFluents)
        : Gecode::Space()
        , mNumberOfTimepoints(numberOfTimepoints)
        , mNumberOfFluents(numberOfFluents)
        , mNumberOfVertices(numberOfTimepoints*numberOfFluents)
        , mGraph(*this, mNumberOfVertices*mNumberOfVertices,0,1)
    {

        templ::solvers::csp::propagators::isPath(*this, mGraph, numberOfTimepoints, numberOfFluents);

        branch(*this, mGraph, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_SPLIT_MIN());
        branch(*this, mGraph, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_SPLIT_MIN());
        branch(*this, mGraph, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_SPLIT_MIN());
    }

    TestSpace(bool share, TestSpace& other)
        : Gecode::Space(share, other)
        , mNumberOfVertices(other.mNumberOfVertices)
    {
        mGraph.update(*this, share, other.mGraph);
    }

    virtual Gecode::Space* copy(bool share)
    {
        return new TestSpace(share, *this);
    }

    std::string toString() const
    {
        std::stringstream ss;
        Gecode::Matrix<Gecode::IntVarArray> graph(mGraph, mNumberOfVertices, mNumberOfVertices);
        for(size_t r = 0; r < mNumberOfVertices; ++r)
        {
            ss << "#row: " << r << " -- ";
            for(size_t c = 0; c < mNumberOfVertices; ++c)
            {
                ss << graph(c,r) << " ";
            }
            ss << std::endl;
        }
        return ss.str();
    }

    void isValid() const
    {
        std::vector<uint32_t> rowSum;
        std::vector<uint32_t> colSum(mNumberOfVertices);
        Gecode::Matrix<Gecode::IntVarArray> graph(mGraph, mNumberOfVertices, mNumberOfVertices);
        for(size_t r = 0; r < mNumberOfVertices; ++r)
        {
            int rowSum = 0;
            for(size_t c = 0; c < mNumberOfVertices; ++c)
            {
                Gecode::IntVar var = graph(c,r);
                if(!var.assigned())
                {
                    throw std::invalid_argument("Can only validate solutions that have been fully assigned");
                }

                Gecode::IntVarValues v(var);
                uint32_t intValue = v.val();
                rowSum += intValue;
                if(rowSum > 1)
                {
                    throw std::runtime_error("Row sum is greater than 0");
                }
                colSum[c] += intValue;
                if(colSum[c] > 1)
                {
                    throw std::runtime_error("Column sum is greater than 0");
                }
            }
        }
    }

};

BOOST_AUTO_TEST_SUITE(propagators)

BOOST_AUTO_TEST_CASE(is_path)
{
    uint32_t numberOfTimepoints = 2;
    uint32_t numberOfFluents = 2;
    TestSpace* testSpace = new TestSpace(numberOfTimepoints, numberOfFluents);
    Gecode::BAB<TestSpace> searchEngine(testSpace);
    TestSpace* best = NULL;
    TestSpace* current = NULL;

    bool foundSolution = false;

    while((current = searchEngine.next()))
    {
        delete best;
        best = current;

        foundSolution = true;

        BOOST_TEST_MESSAGE("BEST:\n" << best->toString());
        BOOST_REQUIRE_NO_THROW(best->isValid());
    }
    BOOST_REQUIRE(foundSolution);
    if(best == NULL)
    {
        delete current;
    }
}

BOOST_AUTO_TEST_CASE(is_path_5_x_1)
{
    uint32_t numberOfTimepoints = 5;
    uint32_t numberOfFluents = 1;
    TestSpace* testSpace = new TestSpace(numberOfTimepoints, numberOfFluents);
    Gecode::BAB<TestSpace> searchEngine(testSpace);
    TestSpace* best = NULL;
    TestSpace* current = NULL;
    bool foundSolution = false;

    while((current = searchEngine.next()))
    {
        delete best;
        best = current;

        foundSolution = true;

        BOOST_TEST_MESSAGE("BEST:\n" << best->toString());
        BOOST_REQUIRE_NO_THROW(best->isValid());
    }
    BOOST_REQUIRE(foundSolution);
    if(best == NULL)
    {
        delete current;
    }
}

BOOST_AUTO_TEST_SUITE_END()
