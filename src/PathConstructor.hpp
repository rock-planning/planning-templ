#ifndef TEMPL_PATH_CONSTRUCTOR_HPP
#define TEMPL_PATH_CONSTRUCTOR_HPP

#include <graph_analysis/algorithms/DFSVisitor.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/Role.hpp>

namespace templ {

class PathConstructor : public graph_analysis::algorithms::DFSVisitor
{
public:
    typedef shared_ptr<PathConstructor> Ptr;

    PathConstructor(const Role& role);

    virtual ~PathConstructor();

    bool invalidTransition(graph_analysis::Edge::Ptr edge);

    /**
     * Override the existing function
     */
    void discoverVertex(graph_analysis::Vertex::Ptr& vertex);

    const std::vector<graph_analysis::Vertex::Ptr>& getPath() const { return mPath; }

private:
    Role mRole;

    std::vector<graph_analysis::Vertex::Ptr> mPath;
};

} // end namespace templ
#endif // TEMPL_PATH_CONSTRUCTOR_HPP
