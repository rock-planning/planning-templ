#ifndef TEMPL_LOGGER_HPP
#define TEMPL_LOGGER_HPP

#include <string>
#include <base/Time.hpp>
#include "../SharedPtr.hpp"

namespace templ {
namespace utils {

class Mission;

/**
 * Logger class which allow to provide session based
 * logging
 */
class Logger
{
public:
    typedef shared_ptr<Logger> Ptr;

    Logger(const base::Time& time = base::Time::now()
            , const std::string& baseDirectory = "/tmp"
            , bool useSessions = true);

    /**
     *
     */
    std::string filename(const std::string& filename) const;


    const base::Time& getTime() const { return mTime; }

    void setBaseDirectory(const std::string& baseDir) { mBaseDirectory = baseDir; }
    const std::string& getBaseDirectory() const { return mBaseDirectory; }

    /**
     * This allow to use session ids within the filename
     * session ids will be incremented using incrementSessionId
     * \see disableSession
     * \see incrementSessionId
     */
    void enableSessions() { mUseSessions = true; }
    /**
     * Disable the use of session ids within the filename
     * (this can also be temporary)
     */
    void disableSessions() { mUseSessions = false; }
    void incrementSessionId() { ++mSessionId; }

    /**
     * Get the current session id
     */
    uint32_t getSessionId() const { return mSessionId; }

    std::string getBasePath() const;

private:
    base::Time mTime;
    std::string mBaseDirectory;
    bool mUseSessions;
    uint32_t mSessionId;
};

} // end namespace utils
} // end namespace templ
#endif // TEMPL_LOGGER_HPP
