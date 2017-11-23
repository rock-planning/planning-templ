#include "Logger.hpp"
#include <sstream>
#include <boost/filesystem.hpp>
#include <base-logging/Logging.hpp>

namespace templ {

Logger::Logger(const base::Time& time, const std::string& baseDirectory, bool useSessions)
    : mTime(time)
    , mBaseDirectory(baseDirectory)
    , mUseSessions(useSessions)
    , mSessionId(0)
{
}

std::string Logger::filename(const std::string& filename) const
{
    std::stringstream ss;
    ss << mBaseDirectory << "/" << mTime.toString(base::Time::Seconds, "%Y%m%d_%H%M%S") << "-templ/";
    if(mUseSessions)
    {
        ss << "/" << mSessionId;
    }
    if(boost::filesystem::create_directories( boost::filesystem::path(ss.str())) )
    {
        LOG_DEBUG_S << "Created directory: '" << ss.str() << "'";
    }

    ss << "/" << filename;
    return ss.str();
}

std::string Logger::getBasePath() const
{
    std::stringstream ss;
    ss << mBaseDirectory << "/" << mTime.toString(base::Time::Seconds, "%Y%m%d_%H%M%S") << "-templ/";
    return ss.str();
}


} // end namespace templ
