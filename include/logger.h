#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <instance.h>

// Define logging methods
enum class LogMethod {
    ROS_INFO,
    COUT,
    FILE,
    NONE
};

enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    HLINFO = 2,
    WARN = 3,
    ERROR = 4,
};

class Logger {
public:
    // Delete copy constructor and assignment operator to prevent copying
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Access the singleton instance of the Logger
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    };

    void setMethod(LogMethod newMethod);

    void setLogLevel(LogLevel level);

    void setLogLevel(const std::string &level);

    void log(const std::string& message, LogLevel level);
    void log(const RobotPose& pose, LogLevel level);
    void log(const RobotTrajectory& traj, LogLevel level);

private:
    LogMethod method;

    LogLevel logLevel = LogLevel::INFO;

    std::ofstream fileStream;
    std::mutex mtx; // For thread safety

    // Private constructor for singleton
    Logger();
};

// Global functions for convenience
void setLogMethod(LogMethod method);

void setLogLevel(LogLevel level);

void setLogLevel(const std::string &level);

void log(const std::string& message, LogLevel level = LogLevel::INFO);

void log(const RobotPose& pose, LogLevel level = LogLevel::INFO);

void log(const RobotTrajectory& traj, LogLevel level = LogLevel::INFO);

void log(const MRTrajectory& traj, LogLevel level = LogLevel::INFO);

void logProgressFileStart(const std::string& filename);

void logProgressFileAppend(const std::string &filename, const std::string& task, double makespan_pre, double makespan_post);

void logProgressFileAppend(const std::string &filename, const std::string& task, double makespan_pre, double makespan_post, double pathlen);

void logProgressFileAppend(const std::string &filename, const std::string &start_pose, const std::string& goal_pose, double makespan_pre, double makespan_post);

void logProgressFileAppend(const std::string &filename, const std::string &start_pose, const std::string& goal_pose, double plan_time, double makespan_pre, double makespan_post);

void logProgressFileAppend(const std::string &filename, const std::string &start_pose, const std::string& goal_pose, double plan_time, 
        double flowtime_pre, double flowtime_post, double makespan_pre, double makespan_post);

#endif // LOGGER_H