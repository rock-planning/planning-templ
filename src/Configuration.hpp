#ifndef TEMPL_CONFIGURATION_HPP
#define TEMPL_CONFIGURATION_HPP

#include <string>
#include <map>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <base-logging/Logging.hpp>
#include <algorithm>

namespace templ {

class Configuration
{
public:
    /**
     * Default configuration
     */
    Configuration(const std::string& path = "");

    /**
     *
     * \throw std::invalid_argument if key does not exist
     */
    const std::string& getValue(const std::string& key) const;

    template<typename T>
    T getValueAs(const std::string& key, const T& defaultValue = T()) const
    {
        try {
            if(!key.empty())
            {
                return boost::lexical_cast<T>( getValue(key) );
            }
        } catch(const std::invalid_argument& e)
        {
            LOG_DEBUG_S << e.what();
        }

        return defaultValue;
    }
    std::string toString() const;

private:
    void loadXML(const std::string& path);

    std::map<std::string, std::string> mProperties;


};

template<>
bool Configuration::getValueAs(const std::string& key, const bool& defaultValue) const;

} // end namespace templ
#endif // TEMPL_CONFIGURATION_HPP
