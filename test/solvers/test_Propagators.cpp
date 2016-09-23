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
        Gecode::Matrix<Gecode::IntVarArray> graph(mGraph, mNumberOfVertices, mNumberOfVertices);
        for(size_t r = 0; r < mNumberOfVertices; ++r)
        {
            uint32_t location = r%mNumberOfFluents;
            uint32_t time = (r-location)/mNumberOfFluents;
            ss << "l" << location <<"-t" << time << ": ";
            for(size_t c = 0; c < mNumberOfVertices; ++c)
            {
                ss << graph(c,r) << " ";
            }
            ss << std::endl;
        }
        return ss.str();
    }

    uint32_t getValue(uint32_t col, uint32_t row) const
    {
        Gecode::Matrix<Gecode::IntVarArray> graph(mGraph, mNumberOfVertices, mNumberOfVertices);
        Gecode::IntVar var = graph(col,row);
        if(!var.assigned())
        {
            throw std::invalid_argument("Can only validate solutions that have been fully assigned");
        }

        Gecode::IntVarValues v(var);
        return v.val();
    }

    uint32_t sumOf(uint32_t fromCol, uint32_t fromRow, uint32_t toCol, uint32_t toRow) const
    {
        uint32_t value = 0;
        for(size_t c = fromCol; c <= toCol; ++c)
        {
            for(size_t r = fromRow; r <= toRow; ++r)
            {
                value += getValue(c,r);
            }
        }
        return value;
    }

    void isValid() const
    {
        std::vector<uint32_t> rowSum;
        std::vector<uint32_t> colSum(mNumberOfVertices);

        for(size_t r = 0; r < mNumberOfVertices; ++r)
        {
            for(size_t c = 0; c < mNumberOfVertices; ++c)
            {
                uint32_t value = getValue(c,r);
                if(value == 1) // there is a transition to row with index of col
                {
                    uint32_t outgoingEdge = sumOf(c+1, c ,mNumberOfVertices -1, c);
                    if(!outgoingEdge)
                    {
                        std::stringstream ss;
                        ss << "row/col: " << r << "/" << c;
                        if( sumOf(c+1, c, mNumberOfVertices -1, mNumberOfVertices -1) != 0)
                        {
                            throw std::runtime_error("There is a transitions that skips one or more timepoints at: " + ss.str());
                        }
                    }

                }
            }
        }

        // No parallel activity
        for(size_t i = 0; i < mNumberOfVertices; i = i + mNumberOfFluents)
        {
            if( sumOf(0,i,mNumberOfVertices-1,i+mNumberOfFluents-1) > 1)
            {
                throw std::runtime_error("Row sum for the same timepoint is greater than 1");
            }
            if( sumOf(i,0, i+mNumberOfFluents-1,mNumberOfVertices-1) > 1)
            {
                throw std::runtime_error("Column sum for the same timepoint is greater than 1");
            }
        }
    }

};

void run_test(uint32_t numberOfTimepoints, uint32_t numberOfFluents, const std::string& tag)
{
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

        try {
            best->isValid();
        } catch(const std::exception& e)
        {
            BOOST_REQUIRE_MESSAGE(false, "\n" << best->toString() << "\n" << tag << " Solution not valid: " << e.what());
        }
    }
    BOOST_REQUIRE_MESSAGE(foundSolution, "Found solution for " << tag);
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
