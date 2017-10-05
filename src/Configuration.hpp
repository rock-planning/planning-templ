#ifndef TEMPL_CONFIGURATION_HPP
#define TEMPL_CONFIGURATION_HPP

#include <string>
#include <map>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <base-logging/Logging.hpp>

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
    T getValueAsNumeric(const std::string& key, T defaultValue = T())
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

} // end namespace templ
#endif // TEMPL_CONFIGURATION_HPP
