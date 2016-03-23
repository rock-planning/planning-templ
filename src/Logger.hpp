#ifndef TEMPL_LOGGER_HPP
#define TEMPL_LOGGER_HPP

#include <string>
#include <base/Time.hpp>
#include <templ/SharedPtr.hpp>

namespace templ {

class Mission;

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

    std::string getBasePath() const;

    /**
     * Save the mission and the organization model files
     */
    void saveInputData(const Mission& mission) const;

private:
    base::Time mTime;
    std::string mBaseDirectory;
    bool mUseSessions;
    uint32_t mSessionId;
};

} // end namespace templ
#endif // TEMPL_LOGGER_HPP
