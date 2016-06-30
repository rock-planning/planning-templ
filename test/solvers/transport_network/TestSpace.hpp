#include <templ/solvers/csp/TemporallyExpandedGraph.hpp>
#include <vector>

namespace templ {
namespace test {

class TestSpace : public Gecode::Space
{
    Gecode::IntVarArray mNetwork;
    size_t mNumberOfFluents;
    size_t mNumberOfTimepoints;
public:
    TestSpace(const std::vector<bool>& network, size_t numberOfTimepoints, size_t numberOfFluents);

    virtual ~TestSpace() {}

    TestSpace(bool share, TestSpace& s);

    virtual Gecode::Space* copy(bool share);

    void print(std::ostream& os) const;
};


} // end test
} // end templ
