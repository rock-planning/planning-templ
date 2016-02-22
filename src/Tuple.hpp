#ifndef TEMPL_TUPLE_HPP
#define TEMPL_TUPLE_HPP

#include <templ/SharedPtr.hpp>
#include <graph_analysis/Vertex.hpp>

namespace templ {

template<typename A, typename B>
class Tuple : public graph_analysis::Vertex
{
public:
    typedef shared_ptr< Tuple<A,B> > Ptr;
    typedef shared_ptr<A> APtr;
    typedef shared_ptr<B> BPtr;

    Tuple(const APtr& a, const BPtr& b)
        : mpA(a)
        , mpB(b)
    {}

    APtr first() const { return mpA; }
    BPtr second() const { return mpB; }

    virtual std::string getClassName() const { return "templ::Tuple"; }
    virtual std::string toString() const { return first()->toString() + "-" + second()->toString(); }

protected:
    graph_analysis::Vertex* getClone() const { return new Tuple(*this); }

    APtr mpA;
    BPtr mpB;
};

} // end namespace templ
#endif // TEMPL_TUPLE_HPP
