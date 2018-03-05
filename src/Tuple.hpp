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

    Tuple()
        : mA()
        , mB()
    {}

    Tuple(const A& a, const B& b)
        : mA(a)
        , mB(b)
    {}

    A first() const { return mA; }
    B second() const { return mB; }

    void setFirst(const A& a) { mA = a; }
    void setSecond(const B& b) { mB = b; }

    std::string getClassName() const override { return "templ::Tuple"; }

    std::string toString() const override { return toString(0); }

    std::string toString(uint32_t indent = 0) const override
    {
        std::string hspace (indent,' ');
        std::stringstream ss;
        ss << hspace << "Tuple:" << std::endl;
        ss << hspace << "    a:" << std::endl;
        if( tuple_get_pointer(mA) )
        {
            ss << tuple_get_pointer(mA)->toString(indent + 8);
        } else {
            ss << "<NULL>";
        }
        ss << std::endl;
        ss << hspace << "    b:" << std::endl;
        if(tuple_get_pointer(mB))
        {
            ss << tuple_get_pointer(mB)->toString( indent + 8);
        } else {
            ss << "<NULL>";
        }
        return ss.str();
    }

    bool operator==(const Tuple<A,B>& other) const
    {
        return mA == other.mA
            && mB == other.mB;
    }

    bool operator<(const Tuple<A,B>& other) const
    {
        if(mA < other.mA)
        {
            return true;
        } else if(mA == other.mA)
        {
            return mB < other.mB;
        }
        return false;
    }

protected:
    graph_analysis::Vertex* getClone() const { return new Tuple(*this); }

    A mA;
    B mB;
};

} // end namespace templ
#endif // TEMPL_TUPLE_HPP
