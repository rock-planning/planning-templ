#include <boost/test/unit_test.hpp>
#include "../../src/solvers/csp/propagators/IsPath.hpp"
#include <gecode/search.hh>
#include <gecode/minimodel.hh>

class TestSpace : public Gecode::Space
{
    uint32_t mNumberOfTimepoints;
    uint32_t mNumberOfFluents;

    uint32_t mNumberOfVertices;

    Gecode::SetVarArray mGraph;

public:
    TestSpace(size_t numberOfTimepoints, size_t numberOfFluents)
        : Gecode::Space()
        , mNumberOfTimepoints(numberOfTimepoints)
        , mNumberOfFluents(numberOfFluents)
        , mNumberOfVertices(numberOfTimepoints*numberOfFluents)
        , mGraph(*this, mNumberOfVertices, Gecode::IntSet::empty, Gecode::IntSet(0,mNumberOfVertices-1), 0,1)
    {

        // get full path length
        templ::solvers::csp::propagators::isPath(*this, mGraph, numberOfTimepoints, numberOfFluents, numberOfTimepoints-1);

        branch(*this, mGraph, Gecode::SET_VAR_MAX_MAX(), Gecode::SET_VAL_MAX_EXC());
        //branch(*this, mGraph, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_SPLIT_MIN());
        //branch(*this, mGraph, Gecode::INT_VAR_NONE(), Gecode::INT_VAL_SPLIT_MIN());
    }

    TestSpace(bool share, TestSpace& other)
        : Gecode::Space(share, other)
        , mNumberOfVertices(other.mNumberOfVertices)
        , mNumberOfFluents(other.mNumberOfFluents)
        , mNumberOfTimepoints(other.mNumberOfTimepoints)
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
        for(int r = 0; r < mGraph.size(); ++r)
        {
            for(Gecode::SetVarGlbValues i(mGraph[r]); i(); ++i)
            {
                ss << r << " --> " << i.val() << std::endl;
            }
        }
        return ss.str();
    }

    uint32_t getValue(uint32_t idx) const
    {
        Gecode::SetVar var = mGraph[idx];
        if(!var.assigned())
        {
            throw std::invalid_argument("Can only validate solutions that have been fully assigned");
        }

        Gecode::SetVarGlbValues v(var);
        return v.val();
    }

    void isValid() const
    {
    }

};

void run_test(uint32_t numberOfTimepoints, uint32_t numberOfFluents, const std::string& tag)
{
    TestSpace* testSpace = new TestSpace(numberOfTimepoints, numberOfFluents);
    Gecode::BAB<TestSpace> searchEngine(testSpace);
    TestSpace* best = NULL;
    TestSpace* current = NULL;
    bool foundSolution = false;

    BOOST_TEST_MESSAGE("RUN SEARCH");
    while((current = searchEngine.next()))
    {
        delete best;
        best = current;

        try {
            best->isValid();
            BOOST_REQUIRE_MESSAGE(true, "Found solution for " << tag << " -- " << best->toString());
            break;
        } catch(const std::exception& e)
        {
            BOOST_REQUIRE_MESSAGE(false, "\n" << best->toString() << "\n" << tag << " Solution not valid: " << e.what());
        }
    }
    if(best == NULL)
    {
        delete current;
    }
}

BOOST_AUTO_TEST_SUITE(propagators)

BOOST_AUTO_TEST_CASE(is_path)
{
    run_test(2,2, "2x2");
}

BOOST_AUTO_TEST_CASE(is_path_5_x_1)
{
    run_test(5,1,"5x1");
}

BOOST_AUTO_TEST_CASE(is_path_3_x_2)
{
    run_test(3,2,"3x2");
}

BOOST_AUTO_TEST_CASE(is_path_5_x_4)
{
    run_test(5,4,"5x4");
}

BOOST_AUTO_TEST_SUITE_END()
