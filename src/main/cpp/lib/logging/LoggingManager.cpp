#include "lib/logging/LoggingManager.h"


LoggingManager* LoggingManager::instance = nullptr;

LoggingManager* LoggingManager::GetInstance()
{
    if (instance == nullptr) 
    {
        instance = new LoggingManager();
    }

    return instance;
}

void LoggingManager::Log()
{
    for (auto logger : LoggerFunctions)
    {
        auto logfunc = *logger;

        logfunc();
    }
}

void LoggingManager::AddLoggerFunction(std::function<void()>* loggerFunction)
{
    LoggerFunctions.push_back(loggerFunction);
}