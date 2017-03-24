#ifndef TEMPL_TUPLE_HPP
#define TEMPL_TUPLE_HPP

#include <templ/SharedPtr.hpp>
#include <graph_analysis/Vertex.hpp>
#include <sstream>

namespace templ {

/// Allow toString() method of Tuple to handle pointer and value type
template<typename T>
const T* tuple_get_pointer(const T& obj) { return &obj; }

template<typename T>
const T* tuple_get_pointer(const shared_ptr<T>& obj) { return obj.get(); }

/**
 * A Tuple represents a Vertex that can store information tuples
 */
template<typename A, typename B>
class Tuple : public graph_analysis::Vertex
{
public:
    typedef shared_ptr< Tuple<A,B> > Ptr;
    typedef A a_t;
    typedef B b_t;

    Tuple(const A& a, const B& b)
        : mA(a)
        , mB(b)
    {}

    A first() const { return mA; }
    B second() const { return mB; }

    virtual std::string getClassName() const { return "templ::Tuple"; }
    virtual std::string toString() const
    {
        std::stringstream ss;
        ss << tuple_get_pointer(mA)->toString() << "-" << tuple_get_pointer(mB)->toString();
        return ss.str();
    }

protected:
    graph_analysis::Vertex* getClone() const { return new Tuple(*this); }

    A mA;
    B mB;
};

} // end namespace templ
#endif // TEMPL_TUPLE_HPP
