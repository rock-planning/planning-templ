#ifndef TEMPL_ROLE_INFO_VERTEX_HPP
#define TEMPL_ROLE_INFO_VERTEX_HPP

#include <graph_analysis/Vertex.hpp>
#include "RoleInfo.hpp"

namespace templ {

/**
 * A vertex definition to allow storing RoleInfo using the graph base
 * representation
 */
class RoleInfoVertex: public graph_analysis::Vertex, public virtual RoleInfo
{
public:
    typedef shared_ptr<RoleInfoVertex> Ptr;
    typedef shared_ptr<RoleInfoVertex> PtrList;

    virtual ~RoleInfoVertex() {}
    std::string getClassName() const override { return "RoleInfoVertex"; }
    std::string toString() const override { return RoleInfo::toString(); }

protected:
    graph_analysis::Vertex* getClone() const override { return new RoleInfoVertex(*this); }
};

} // end namespace templ
#endif // TEMPL_ROLE_INFO_VERTEX_HPP
