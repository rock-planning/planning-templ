#ifndef TEMPL_LOGGING_HPP
#define TEMPL_LOGGING_HPP

#include <string>
#include <base/Time.hpp>

namespace templ {

class Logging
{
public:
    Logging(const base::Time& time = base::Time::now()
            , const std::string& baseDirectory = "/tmp"
            , bool useSessions = true);

    /**
     * 
     */
    std::string filename(const std::string& filename) const;


    const base::Time& getTime() const { return mTime; }
    const std::string& getBaseDirectory() const { return mBaseDirectory; }
    void enableSessions() { mUseSessions = true; }
    void disableSessions() { mUseSessions = false; }
    void incrementSessionId() { ++mSessionId; }

private:
    base::Time mTime;
    std::string mBaseDirectory;
    bool mUseSessions;
    uint32_t mSessionId;
};

} // end namespace templ
#endif // TEMPL_LOGGING_HPP
