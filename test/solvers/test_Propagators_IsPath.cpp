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
        templ::solvers::csp::propagators::isPath(*this, mGraph, "test-path", numberOfTimepoints, numberOfFluents, numberOfTimepoints-1);
        branch(*this, mGraph, Gecode::SET_VAR_MAX_MAX(), Gecode::SET_VAL_MAX_EXC());
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
        ss << "Solution (" << mGraph.size() << ")" << std::endl;
        for(int r = 0; r < mGraph.size(); ++r)
        {
            ss << r << " --> " << mGraph[r] << std::endl;
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

    BOOST_TEST_MESSAGE("Run search");
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

BOOST_AUTO_TEST_CASE(check_valid_path)
{
    using namespace templ::solvers::csp::propagators;
    typedef std::pair<int, bool> wp;

    size_t start, end;

    std::vector< std::pair<int, bool> > path;
    path.push_back(wp(-1,true));
    BOOST_REQUIRE_MESSAGE( IsPath::isValidWaypointSequence(path, start, end), "Single unassigned row is an ok path: from: " << start << " to: " << end);
    path.push_back(wp(1,true));
    BOOST_REQUIRE_MESSAGE( IsPath::isValidWaypointSequence(path, start, end), "Single unassigned row and single assigned row is an ok path: from: " << start << " to: " << end );
    path.push_back(wp(1,true));
    BOOST_REQUIRE_MESSAGE( IsPath::isValidWaypointSequence(path, start, end), "Single unassigned row and 2 assigned rows is an ok path: from: " << start << " to: " << end);
    path.push_back(wp(-1,true));
    BOOST_REQUIRE_MESSAGE( IsPath::isValidWaypointSequence(path, start, end), "Single unassigned row and 2 assigned rows, and single unassigned is an ok path: from: " << start << " to: " << end );
    path.push_back(wp(1,true));
    BOOST_REQUIRE_MESSAGE(!IsPath::isValidWaypointSequence(path, start, end), "Path with gap is not a valid sequence" );

}

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

BOOST_AUTO_TEST_CASE(is_path_20_x_20)
{
    run_test(20,20,"20x20");
}

BOOST_AUTO_TEST_CASE(is_path_100_x_100)
{
    run_test(100,100,"100x100");
}

BOOST_AUTO_TEST_SUITE_END()
