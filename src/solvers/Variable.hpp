#ifndef TEREP_SOLVERS_VARIABLE_HPP
#define TEREP_SOLVERS_VARIABLE_HPP

#include <graph_analysis/Vertex.hpp>

namespace terep {
namespace solvers {

class Variable : public graph_analysis::Vertex
{
public:
    typedef boost::shared_ptr<Variable> Ptr;

    virtual std::string getClassName() const { return "Variable"; }

    virtual std::string toString() const;
};

} // end namespace solvers
} // end namespace terep
#endif // TEREP_SOLVERS_VARIABLE_HPP
