#ifndef TEMPL_UTILS_INDEX_HPP
#define TEMPL_UTILS_INDEX_HPP

#include <vector>
#include <algorithm>
#include <stdexcept>

namespace templ {
namespace utils {

class Index
{
public:
    template<typename T>
    static size_t getIndex(const std::vector<T>& list, const T& searchItem, const std::string& errorMsg)
    {
        typename std::vector<T>::const_iterator cit = std::find(list.cbegin(), list.cend(), searchItem);
        if(cit != list.cend())
        {
            return std::distance(cit, list.cbegin());
        }

        throw std::invalid_argument("templ::utils::Index::getIndex failed -- msg: '" + errorMsg +"'");

    }

};


} // end namespace utils
} // end namespace templ
#endif // TEMPL_UTILS_INDEX_HPP
