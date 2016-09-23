#ifndef TEMPL_SHARED_PTR_HPP
#define TEMPL_SHARED_PTR_HPP

#include <vector>

#if __cplusplus <= 199711L
#define USE_BOOST_SHARED_PTR
#endif

#ifdef USE_BOOST_SHARED_PTR
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#else
#include <memory>
#include <functional>
#endif

namespace templ
{
#ifdef USE_BOOST_SHARED_PTR
    using ::boost::shared_ptr;
    using ::boost::make_shared;
    using ::boost::dynamic_pointer_cast;
    using ::boost::static_pointer_cast;
    using ::boost::function1;
#else
    using ::std::shared_ptr;
    using ::std::make_shared;
    using ::std::dynamic_pointer_cast;
    using ::std::static_pointer_cast;
    template <class T, class U>
    using function1 = ::std::function<T(U)>;
#endif

    // Convert pointer array from a given type to target type
    // e.g. to use base type as function argument
    template<typename toT, typename fromT>
    std::vector< shared_ptr<toT> > toPtrList(const std::vector< shared_ptr<fromT> >& pointerList)
    {
        typedef std::vector< shared_ptr<toT> > ToPtrList;
        typedef std::vector< shared_ptr<fromT> > FromPtrList;
        ToPtrList ptrList;
        typename FromPtrList::const_iterator cit = pointerList.begin();
        for(; cit != pointerList.end(); ++cit)
        {
            shared_ptr<toT> ptr = dynamic_pointer_cast<toT>(*cit);
            if(!ptr)
            {
                throw std::invalid_argument("convertToSymbolPtrList: "
                        " type in list does not inherit from target type");
            }
            ptrList.push_back(ptr);
        }
        return ptrList;
    }
}

#endif // TEMPL_SHARED_PTR_HPP
