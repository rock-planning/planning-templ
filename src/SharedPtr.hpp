#ifndef TEMPL_SHARED_PTR_HPP
#define TEMPL_SHARED_PTR_HPP

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
}

#endif // TEMPL_SHARED_PTR_HPP
