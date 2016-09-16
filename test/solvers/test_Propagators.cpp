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
            for(size_t c = 0; c < mNumberOfVertices; ++c)
            {
                ss << graph(c,r) << " ";
            }
            ss << std::endl;
        }
        return ss.str();
    }

};

BOOST_AUTO_TEST_SUITE(propagators)

BOOST_AUTO_TEST_CASE(is_path)
{
    TestSpace* testSpace = new TestSpace(2,2);
    Gecode::BAB<TestSpace> searchEngine(testSpace);
    TestSpace* best = NULL;
    TestSpace* current = NULL;

    while((current = searchEngine.next()))
    {
        delete best;
        best = current;

        BOOST_TEST_MESSAGE("BEST:\n" << best->toString());
    }
    if(best == NULL)
    {
        delete current;
    }
}

BOOST_AUTO_TEST_SUITE_END()
