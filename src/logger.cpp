#include "logger.h"

// Define ANSI escape codes for colors
const std::string RED = "\033[31m";
const std::string YELLOW = "\033[33m";
const std::string RESET = "\033[0m";

void Logger::setMethod(LogMethod newMethod) {
    std::lock_guard<std::mutex> lock(mtx);
    method = newMethod;
    if (method == LogMethod::FILE) {
        fileStream.open("log.txt", std::ofstream::out | std::ofstream::app);
        if (!fileStream.is_open()) {
            std::cerr << "Failed to open log file!" << std::endl;
            method = LogMethod::COUT; // Fallback to cout if file opening fails
        }
    } else if (fileStream.is_open()) {
        fileStream.close();
    }
}

void Logger::setLogLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(mtx);
    logLevel = level;
}

void Logger::setLogLevel(const std::string &level) {
    if (level == "debug") {
        setLogLevel(LogLevel::DEBUG);
    }
    else if (level == "info") {
        setLogLevel(LogLevel::INFO);
    }
    else if (level == "hlinfo") {
        setLogLevel(LogLevel::HLINFO);
    }
    else if (level == "warn") {
        setLogLevel(LogLevel::WARN);
    }
    else if (level == "error") {
        setLogLevel(LogLevel::ERROR);
    }
    else {
        std::cerr << "Invalid log level: " << level << std::endl;
    }
}

void Logger::log(const std::string& message, LogLevel level) {
    if (level < logLevel) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx);
    switch (method) {
        case LogMethod::ROS_INFO:
            // ROS_INFO("%s", message.c_str()); // Uncomment this if using ROS
            break;
        case LogMethod::COUT:
            if (level == LogLevel::ERROR) {
                std::cerr << RED << message << RESET << std::endl;
            }
            else if (level == LogLevel::WARN) {
                std::cerr << YELLOW << message << RESET << std::endl;
            }
            else {
                std::cout << message << std::endl;
            }
            break;
        case LogMethod::FILE:
            if (fileStream.is_open()) {
                fileStream << message << std::endl;
            }
            break;
        case LogMethod::NONE:
            // Do nothing
            break;
    }
}

void Logger::log(const RobotPose& pose, LogLevel level) {
    if (level < logLevel) {
        return;
    }
    assert(pose.joint_values.size() > 0); // Ensure the pose has joint values
    std::string message = "Robot " + pose.robot_name + " (" + std::to_string(pose.robot_id) + "), joint: ";
    for (const auto& value : pose.joint_values) {
        message += std::to_string(value) + " ";
    }
    log(message, level);
}

void Logger::log(const RobotTrajectory& traj, LogLevel level) {
    if (level < logLevel) {
        return;
    }
    std::string message = "Robot " + std::to_string(traj.robot_id) + ", trajectory: \n";
    assert(traj.trajectory.size() == traj.times.size());
    for (size_t i = 0; i < traj.trajectory.size(); ++i) {
        if (traj.act_ids.size() > i) {
            message += "Act ID: " + std::to_string(traj.act_ids[i]) + " ";
        }
        message += "Pose " + std::to_string(i) + " (t=" + std::to_string(traj.times[i]) + "): ";
        for (const auto& value : traj.trajectory[i].joint_values) {
            message += std::to_string(value) + " ";
        }
        message += "\n";
    }
    log(message, level);
}

Logger::Logger() : method(LogMethod::COUT), logLevel(LogLevel::INFO) {}

// Global functions for convenience
void setLogMethod(LogMethod method) {
    Logger::getInstance().setMethod(method);
}

void setLogLevel(LogLevel level) {
    Logger::getInstance().setLogLevel(level);
}

void setLogLevel(const std::string &level) {
    Logger::getInstance().setLogLevel(level);
}

void log(const std::string& message, LogLevel level) {
    Logger::getInstance().log(message, level);
}

void log(const RobotPose& pose, LogLevel level) {
    Logger::getInstance().log(pose, level);
}

void log(const RobotTrajectory& traj, LogLevel level) {
    Logger::getInstance().log(traj, level);
}

void log(const MRTrajectory& traj, LogLevel level) {
    for (const auto& robot_traj : traj) {
        log(robot_traj, level);
    }
}

void logProgressFileStart(const std::string& filename) {
    if (filename != "") {
        std::ofstream ofs(filename, std::ofstream::out);
        ofs << "start_pose,goal_pose,flowtime_pre,makespan_pre,flowtime_post,makespan_post,t_plan,t_init,t_shortcut,t_mcp,t_check," <<
            "n_check,n_valid,n_type2_pre,n_colcheck_pre,n_type2_post,n_colcheck_post,pathlen,t_wait,norm_isj,dir_consistency," <<
            "n_comp,n_pp,n_path,n_v_comp,n_v_pp,n_v_path"<< std::endl;
        ofs.close();
    }
}

void logProgressFileAppend(const std::string &filename, const std::string& task, double makespan_pre, double makespan_post) {
    if (filename != "") {
        std::ofstream ofs(filename, std::ofstream::app);
        ofs << task << "," << task << ",," << makespan_pre << ",," << makespan_post << ",,,,,,,,,,,,,,,,,,,,," << std::endl;
        ofs.close();
    }
}

void logProgressFileAppend(const std::string &filename, const std::string& task, double makespan_pre, double makespan_post, double pathlen) {
    if (filename != "") {
        std::ofstream ofs(filename, std::ofstream::app);
        ofs << task << "," << task << ",," << makespan_pre << ",," << makespan_post << ",,,,,,,,,,,," << pathlen << ",,,,,,,,," << std::endl;
        ofs.close();
    }
}

void logProgressFileAppend(const std::string &filename, const std::string &start_pose, const std::string& goal_pose, double makespan_pre, double makespan_post) {
    if (filename != "") {
        std::ofstream ofs(filename, std::ofstream::app);
        ofs << start_pose << "," << goal_pose << ",," << makespan_pre << ",," << makespan_post << ",,,,,,,,,,,,,,,,,,,,," << std::endl;
        ofs.close();
    }
}

void logProgressFileAppend(const std::string &filename, const std::string &start_pose, const std::string& goal_pose, double plan_time, double makespan_pre, double makespan_post) {
    if (filename != "") {
        std::ofstream ofs(filename, std::ofstream::app);
        ofs << start_pose << "," << goal_pose << ",," << makespan_pre << ",," << makespan_post << "," << plan_time << ",,,,,,,,,,,,,,,,,,,," << std::endl;
        ofs.close();
    }
}

void logProgressFileAppend(const std::string &filename, const std::string &start_pose, const std::string& goal_pose, double plan_time, 
        double flowtime_pre, double flowtime_post, double makespan_pre, double makespan_post) {
    if (filename != "") {
        std::ofstream ofs(filename, std::ofstream::app);
        ofs << start_pose << "," << goal_pose << "," << flowtime_pre << "," << makespan_pre << "," << flowtime_post << "," << makespan_post << "," << plan_time << ",,,,,,,,,,,,,,,,,,,," << std::endl;
        ofs.close();
    }
}