#include "TestSpace.hpp"
#include <gecode/gist.hh>

namespace templ {
namespace test {

TestSpace::TestSpace(const std::vector<bool>& network, size_t numberOfTimepoints, size_t numberOfFluents)
    : Gecode::Space()
    , mNetwork(*this, (numberOfTimepoints*numberOfFluents)*(numberOfTimepoints*numberOfFluents), 0, 1)
    , mNumberOfFluents(numberOfFluents)
    , mNumberOfTimepoints(numberOfTimepoints)
{
    size_t timeFluentsSize = numberOfTimepoints*numberOfFluents;
    Gecode::Matrix<Gecode::IntVarArray> adjacencyGraph(mNetwork, timeFluentsSize, timeFluentsSize);

    for(size_t col = 0; col < timeFluentsSize; ++col)
    {
        for(size_t row = 0; row < timeFluentsSize; ++row)
        {
            size_t index = row*timeFluentsSize + col;
            if( network.at(index) )
            {
                rel(*this, adjacencyGraph(col,row) == 1);
            } else {
                rel(*this, adjacencyGraph(col,row) == 0);
            }
        }
    }

    std::cout << "Print " << mNetwork << std::endl;

    using namespace solvers::csp;
    isPath(*this, mNetwork, mNumberOfTimepoints, mNumberOfFluents);

    branch(*this, mNetwork, Gecode::INT_VAR_SIZE_MAX(), Gecode::INT_VAL_SPLIT_MIN());
    branch(*this, mNetwork, Gecode::INT_VAR_MIN_MIN(), Gecode::INT_VAL_SPLIT_MIN());

//    Gecode::Gist::Print<TestSpace> p("Print solution");
//    Gecode::Gist::Options o;
//    o.inspect.click(&p);
//    Gecode::Gist::bab(this, o);
}

void TestSpace::print(std::ostream& os) const
{
    os << mNetwork << std::endl;
}

TestSpace::TestSpace(bool share, TestSpace& other)
    : Gecode::Space(share, other)
{
    mNetwork.update(*this, share, other.mNetwork);
}

Gecode::Space* TestSpace::copy(bool share)
{
    return new TestSpace(share, *this);
}

} // end namespace test
} // end namespace templ
