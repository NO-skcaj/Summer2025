#pragma once

#include <vector>
#include <functional>


/// @brief Class to manage the logging of data
class LoggingManager
{
    public:

        static LoggingManager* GetInstance();

        void Log();

        void AddLoggerFunction(std::function<void()> loggerFunction);

    private:
        LoggingManager() = default;

        std::vector<std::function<void()>> LoggerFunctions;

        static LoggingManager* instance;
};