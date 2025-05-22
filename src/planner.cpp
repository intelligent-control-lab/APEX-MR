#include "planner.h"
#include "instance.h"
#include "logger.h"
#include <boost/algorithm/string.hpp>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/kinematic_constraints/utils.h>


bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    MRTrajectory &solution,
                    bool reset_speed) {
    // Convert a MoveIt plan to a RobotTrajectory
    int numRobots = instance->getNumberOfRobots();
    solution.clear();
    solution.resize(numRobots);
    for (int i = 0; i < numRobots; i++) {
        solution[i].robot_id = i;
    }

    for (int i = 0; i < plan_traj.joint_trajectory.points.size(); i++) {
        int st = 0;
        double timeDilation = 0;
        for (int j = 0; j < numRobots; j++) {
            RobotPose pose = instance->initRobotPose(j);
            assert (st + pose.joint_values.size() <= plan_traj.joint_trajectory.points[i].positions.size());
            for (int k = 0; k < pose.joint_values.size(); k++) {
                pose.joint_values[k] = plan_traj.joint_trajectory.points[i].positions[k + pose.joint_values.size()*j];
            }

            if (i > 0) {
                double dt = plan_traj.joint_trajectory.points[i].time_from_start.toSec() - plan_traj.joint_trajectory.points[i-1].time_from_start.toSec();
                double speed = std::abs(instance->computeDistance(solution[j].trajectory.back(), pose)) / dt;
                if (speed > instance->getVMax(j)) {
                    timeDilation = std::max(timeDilation, speed / instance->getVMax(j));
                }
                else if (speed <= instance->getVMax(j)) {
                    timeDilation = std::max(timeDilation, speed / instance->getVMax(j)); 
                }
            }
            solution[j].trajectory.push_back(pose);
            st += pose.joint_values.size();
        }

        for (int j = 0; j < numRobots; j++) {
            if (i > 0) {
                double dt = plan_traj.joint_trajectory.points[i].time_from_start.toSec() - plan_traj.joint_trajectory.points[i-1].time_from_start.toSec();
                if (reset_speed) {
                    dt = dt * timeDilation;
                }
                solution[j].times.push_back(solution[j].times.back() + dt);
            }
            else {
                solution[j].times.push_back(plan_traj.joint_trajectory.points[i].time_from_start.toSec());
            }
        }
    }

    // for each robot, if the robot is not moving at the end, keep removing the last point
    for (int i = 0; i < numRobots; i++) {
       RobotPose last_pose = solution[i].trajectory.back();
       while (solution[i].trajectory.size() > 1 && instance->computeDistance(solution[i].trajectory[solution[i].trajectory.size()-2], last_pose) < 1e-5) {
           solution[i].trajectory.pop_back();
           solution[i].times.pop_back();
       }
    }

    for (int i = 0; i < numRobots; i++) {
        solution[i].cost = solution[i].times.back();
    }

    return true;
}

bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    int robot_id,
                    RobotTrajectory &solution)
{   
    for (int i = 0; i < plan_traj.joint_trajectory.points.size(); i++) {
        double timeDilation = 0;
        RobotPose pose = instance->initRobotPose(robot_id);
        
        assert (pose.joint_values.size() == plan_traj.joint_trajectory.points[i].positions.size());
        for (int k = 0; k < pose.joint_values.size(); k++) {
            pose.joint_values[k] = plan_traj.joint_trajectory.points[i].positions[k];
        }

        if (i > 0) {
            double dt = plan_traj.joint_trajectory.points[i].time_from_start.toSec() - plan_traj.joint_trajectory.points[i-1].time_from_start.toSec();
            double speed = std::abs(instance->computeDistance(solution.trajectory.back(), pose)) / dt;
            timeDilation = speed / instance->getVMax(robot_id);
        }
        solution.trajectory.push_back(pose);

        if (i > 0) {
            double dt = plan_traj.joint_trajectory.points[i].time_from_start.toSec() - plan_traj.joint_trajectory.points[i-1].time_from_start.toSec();
            dt = dt * timeDilation;
            solution.times.push_back(solution.times.back() + dt);
        }
        else {
            solution.times.push_back(plan_traj.joint_trajectory.points[i].time_from_start.toSec());
        }
    }


    solution.cost = solution.times.back();
    return true;
}

bool saveSolution(std::shared_ptr<PlanInstance> instance,
                  const moveit_msgs::RobotTrajectory &plan_traj,
                  const std::string &file_name)
{
    int numRobots = instance->getNumberOfRobots();

    std::ofstream file(file_name);
    if (!file.is_open()) {
        log("Failed to open file " + file_name + " for writing!", LogLevel::ERROR);
        return false;
    }
    int total_dim = 0;
    for (int j = 0; j < numRobots; j++) {
        total_dim += instance->getRobotDOF(j);
    }
    if (plan_traj.joint_trajectory.points[0].positions.size() != total_dim) {
        log("Invalid plan trajectory size!", LogLevel::ERROR);
        return false;
    }

    file << "time" << ",";

    for (int j = 0; j < numRobots; j++) {
        for (int d = 0; d < instance->getRobotDOF(j); d++) {
            file << "q" << std::to_string(j) << "_" << std::to_string(d) << ",";
        }
    }
    file << std::endl;

    int max_size = plan_traj.joint_trajectory.points.size();
    for (int i = 0; i < max_size; i++) {
        file << plan_traj.joint_trajectory.points[i].time_from_start.toSec() << ",";
        for (int d = 0; d < total_dim; d++) {
            file << plan_traj.joint_trajectory.points[i].positions[d] << ",";
        }
        file << std::endl;
    }
    file.close();

    return true;
}

bool saveSolution(std::shared_ptr<PlanInstance> instance,
                  const MRTrajectory &synced_traj,
                  const std::string &file_name)

{
    int numRobots = instance->getNumberOfRobots();

    std::ofstream file(file_name);
    if (!file.is_open()) {
        log("Failed to open file " + file_name + " for writing!", LogLevel::ERROR);
        return false;
    }
    int total_dim = 0;
    for (int j = 0; j < numRobots; j++) {
        total_dim += instance->getRobotDOF(j);
    }

    file << "time" << ",";

    for (int j = 0; j < numRobots; j++) {
        for (int d = 0; d < instance->getRobotDOF(j); d++) {
            file << "q" << std::to_string(j) << "_" << std::to_string(d) << ",";
        }
    }
    file << std::endl;

    int max_size = 0;
    for (int j = 0; j < numRobots; j++) {
        max_size = std::max(max_size, int(synced_traj[j].times.size()));
    }
    for (int i = 0; i < max_size; i++) {
        double t = -1;
        for (int j = 0; j < numRobots; j++) {
            if (i < synced_traj[j].times.size()) {
                double tj = synced_traj[j].times[i];
                if (t != -1 && (tj != t)) {
                    log("Time is not synched for the plan!", LogLevel::ERROR);
                    return false;
                }
                t = tj;
            }
        }
        file << t << ",";
        for (int j = 0; j < numRobots; j++) {
            for (int d = 0; d < instance->getRobotDOF(j) ; d++) {
                file << synced_traj[j].trajectory[i].joint_values[d] << ",";
            }
        }
        file << std::endl;
    }
    file.close();

    return true;
}

bool loadSolution(std::shared_ptr<PlanInstance> instance,
                 const std::string &file_name,
                 double dt,
                 moveit_msgs::RobotTrajectory &plan_traj)
{
    // assume joint name is already given
    int numRobots = instance->getNumberOfRobots();

    std::ifstream file(file_name);
    if (!file.is_open()) {
        log("Failed to open file " + file_name + " for reading!", LogLevel::ERROR);
        return false;
    }
    
    
    int total_dim = 0;
    for (int j = 0; j < numRobots; j++) {
        total_dim += instance->getRobotDOF(j);
    }
    std::string line;
    std::getline(file, line);
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(","));
    if (tokens.size() != total_dim || plan_traj.joint_trajectory.joint_names.size() != total_dim) {
        log("Invalid plan trajectory size!", LogLevel::ERROR);
        return false;
    }

    plan_traj.joint_trajectory.points.clear();    
   
    int t = 0;
    while (std::getline(file, line)) {
        boost::split(tokens, line, boost::is_any_of(","));
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(total_dim);
        point.velocities.resize(total_dim);
        point.accelerations.resize(total_dim);
        point.time_from_start = ros::Duration(t * dt);
        for (int d = 0; d < total_dim; d++) {
            point.positions[d] = std::stod(tokens[d]);
        }

        plan_traj.joint_trajectory.points.push_back(point);
        t++;
    }
    
    // compute velocities and accelerations with central difference
    auto &points = plan_traj.joint_trajectory.points;
    for (int i = 1; i < points.size() - 1; i++) {
        for (int j = 0; j < total_dim; j++) {
            points[i].velocities[j] = (points[i+1].positions[j] - points[i-1].positions[j]) / (2 * dt);
            points[i].accelerations[j] = (points[i+1].positions[j] - 2 * points[i].positions[j] + points[i-1].positions[j]) / (dt * dt);
        }
    }

    return true;
}


bool loadSolution(std::shared_ptr<PlanInstance> instance,
                 const std::string &file_name,
                 moveit_msgs::RobotTrajectory &plan_traj)
{
    // assume joint name is already given
    int numRobots = instance->getNumberOfRobots();

    std::ifstream file(file_name);
    if (!file.is_open()) {
        log("Failed to open file " + file_name + " for reading!", LogLevel::ERROR);
        return false;
    }
    
    
    int total_dim = 0;
    for (int j = 0; j < numRobots; j++) {
        total_dim += instance->getRobotDOF(j);
    }
    std::string line;
    std::getline(file, line);
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(","));
    // some csv has a extra comma at the end
    if (tokens.back().empty()) {
        tokens.pop_back();
    }
    if (tokens.size() != (total_dim + 1)|| plan_traj.joint_trajectory.joint_names.size() != total_dim) {
        log("Invalid plan trajectory size!", LogLevel::ERROR);
        return false;
    }

    plan_traj.joint_trajectory.points.clear();    
   
    int t = 0;
    while (std::getline(file, line)) {
        boost::split(tokens, line, boost::is_any_of(","));
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(total_dim);
        point.velocities.resize(total_dim);
        point.accelerations.resize(total_dim);
        point.time_from_start = ros::Duration(std::stod(tokens[0]));
        for (int d = 0; d < total_dim; d++) {
            point.positions[d] = std::stod(tokens[d+1]);
        }

        plan_traj.joint_trajectory.points.push_back(point);
        t++;
    }
    std::cout << "Loaded " << t << " points" << std::endl;
    
    // compute velocities and accelerations with central difference
    auto &points = plan_traj.joint_trajectory.points;
    for (int i = 1; i < points.size() - 1; i++) {
        for (int j = 0; j < total_dim; j++) {
            points[i].velocities[j] = (points[i+1].positions[j] - points[i-1].positions[j]) / (points[i+1].time_from_start.toSec() - points[i-1].time_from_start.toSec());
            points[i].accelerations[j] = (points[i+1].positions[j] - 2 * points[i].positions[j] + points[i-1].positions[j]) 
                / ((points[i].time_from_start.toSec() - points[i-1].time_from_start.toSec()) * (points[i+1].time_from_start.toSec() - points[i].time_from_start.toSec()));
        }
    }

    return true;
}

void retimeSolution(std::shared_ptr<PlanInstance> instance,
                    const MRTrajectory &solution,
                    MRTrajectory &retime_solution,
                    double dt)
{
    // retime solution based on maximum velocity, assuming solution has uniform time discretization and the same time for all robots
    // no act id
    int num_robot = instance->getNumberOfRobots();
    retime_solution.clear();

    int count = solution[0].times.size();
    for (int i = 0; i < num_robot; i++) {
        retime_solution.push_back(RobotTrajectory());
        retime_solution[i].robot_id = i;
        if (count != solution[i].times.size()) {
            log("Solution has different number of points for each robot!", LogLevel::ERROR);
        }
        if (solution[i].act_ids.size() > 0) {
            log("Solution is multi-task!", LogLevel::ERROR);
        }
    }

    int step = 0;
    while (step < count) {
        double timeDilation = 0;
        double dt_step;
        if (step > 0) {
            for (int i = 0; i < instance->getNumberOfRobots(); i++) {
                double dist = std::abs(instance->computeDistance(solution[i].trajectory[step], solution[i].trajectory[step - 1]));
                dt_step = solution[i].times[step] - solution[i].times[step - 1];
                double speed = (dist / dt_step);
                timeDilation = std::max(timeDilation, speed / instance->getVMax(i)); 
            }
        }

        // append the point to speedup_traj
        for (int i = 0; i < instance->getNumberOfRobots(); i++) {
            retime_solution[i].trajectory.push_back(solution[i].trajectory[step]);
            if (step > 0) {
                retime_solution[i].times.push_back(retime_solution[i].times.back() + dt_step * timeDilation);
            }
            else {
                retime_solution[i].times.push_back(solution[i].times[step]);
            }
        }
        step ++;
    }

    for (int i = 0; i < num_robot; i++) {
        retime_solution[i].cost = retime_solution[i].times.back();
    }
}

void rediscretizeSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    moveit_msgs::RobotTrajectory &retime_traj,
                    double new_dt)
{
    // Convert a MoveIt plan to a RobotTrajectory
    int numRobots = instance->getNumberOfRobots();
    retime_traj.joint_trajectory.joint_names = plan_traj.joint_trajectory.joint_names;
    retime_traj.joint_trajectory.points.clear();
    
    int numPoints = std::ceil(plan_traj.joint_trajectory.points.back().time_from_start.toSec() / new_dt) + 1;
    retime_traj.joint_trajectory.points.resize(numPoints);
    int ind = 0;
    int total_dim = plan_traj.joint_trajectory.joint_names.size();

    for (int i = 0; i < numPoints; i++) {
        double time = i * new_dt;
        while (ind + 1 < plan_traj.joint_trajectory.points.size() && plan_traj.joint_trajectory.points[ind+1].time_from_start.toSec() <= time) {
            ind++;
        }
        int dof_s = 0;
        for (int j = 0; j < numRobots; j++) {
            RobotPose pose = instance->initRobotPose(j);
            assert ((dof_s + pose.joint_values.size()) <= plan_traj.joint_trajectory.points[ind].positions.size());
            if (ind + 1 == plan_traj.joint_trajectory.points.size()) {
                for (int d = 0; d < instance->getRobotDOF(j); d++) {
                    pose.joint_values[d] = plan_traj.joint_trajectory.points[ind].positions[dof_s + d];
                }
            }
            else {
                RobotPose pose_next = instance->initRobotPose(j);
                RobotPose pose_prev = instance->initRobotPose(j);
                for (int d = 0; d < instance->getRobotDOF(j); d++) {
                    pose_next.joint_values[d] = plan_traj.joint_trajectory.points[ind+1].positions[dof_s + d];
                    pose_prev.joint_values[d] = plan_traj.joint_trajectory.points[ind].positions[dof_s + d];
                }
                double alpha = (time - plan_traj.joint_trajectory.points[ind].time_from_start.toSec()) 
                        / (plan_traj.joint_trajectory.points[ind + 1].time_from_start.toSec() - plan_traj.joint_trajectory.points[ind].time_from_start.toSec());
                pose = instance->interpolate(pose_prev, pose_next, alpha);
            }

            for (int d = 0; d < instance->getRobotDOF(j); d++) {
                retime_traj.joint_trajectory.points[i].positions.push_back(pose.joint_values[d]);
            }

            dof_s += instance->getRobotDOF(j);
        }
        retime_traj.joint_trajectory.points[i].time_from_start = ros::Duration(time);
        retime_traj.joint_trajectory.points[i].velocities.resize(total_dim);
        retime_traj.joint_trajectory.points[i].accelerations.resize(total_dim);

    }

    // compute velocities and accelerations with central difference
    auto &points = retime_traj.joint_trajectory.points;
    for (int i = 1; i < points.size() - 1; i++) {
        for (int j = 0; j < total_dim; j++) {
            points[i].velocities[j] = (points[i+1].positions[j] - points[i-1].positions[j]) / (points[i+1].time_from_start.toSec() - points[i-1].time_from_start.toSec());
            points[i].accelerations[j] = (points[i+1].positions[j] - 2 * points[i].positions[j] + points[i-1].positions[j]) 
                / ((points[i].time_from_start.toSec() - points[i-1].time_from_start.toSec()) * (points[i+1].time_from_start.toSec() - points[i].time_from_start.toSec()));
        }
    }

}

void rediscretizeSolution(std::shared_ptr<PlanInstance> instance,
                        const MRTrajectory &solution,
                        MRTrajectory &retime_solution,
                        double new_dt)
{
    // assuming a single task for each robot
    int numRobots = instance->getNumberOfRobots();
    double maxtime = 0;
    for (int i = 0; i < numRobots; i++) {
        maxtime = std::max(maxtime, solution[i].times.back());
        if (solution[i].act_ids.size() > 0) {
            log("Solution is multi-task!", LogLevel::ERROR);
        }
    }

    retime_solution.clear();
    retime_solution.resize(numRobots);
    for (int i = 0; i < numRobots; i++) {
        retime_solution[i].robot_id = i;
    }

    int numSteps = maxtime / new_dt;
    
    std::vector<int> index(numRobots, 0);
    for (int s = 0; s <= numSteps; s++) {
        double t = s * new_dt;
        for (int i = 0; i < numRobots; i++) {
            while (index[i] + 1 < solution[i].times.size() && solution[i].times[index[i] + 1] <= t) {
                index[i]++;
            }
            if (index[i] + 1 == solution[i].times.size()) {
                // assuming obstacle stays at the end of the trajectory
                retime_solution[i].trajectory.push_back(solution[i].trajectory[index[i]]);
                retime_solution[i].times.push_back(t);
            } else {
                double alpha = (t - solution[i].times[index[i]]) / (solution[i].times[index[i] + 1] - solution[i].times[index[i]]);
                RobotPose pose_i = instance->interpolate(solution[i].trajectory[index[i]], solution[i].trajectory[index[i] + 1], alpha);
                retime_solution[i].trajectory.push_back(pose_i);
                retime_solution[i].times.push_back(t);
            }
        }
    }
    
    for (int i = 0; i < numRobots; i++) {
        retime_solution[i].cost = retime_solution[i].times.back();
    }
    return;
}

void removeWait(std::shared_ptr<PlanInstance> instance,
                        MRTrajectory &solution)
{
    int numRobots = instance->getNumberOfRobots();
    for (int i = 0; i < numRobots; i++)
    {
        int index = solution[i].times.size() - 1;
        while (index >= 1) {
            double dist = instance->computeDistance(solution[i].trajectory[index - 1], solution[i].trajectory[index]);
            if (dist < 1e-5) {
                solution[i].trajectory.erase(solution[i].trajectory.begin() + index);
                solution[i].times.erase(solution[i].times.begin() + index);
            }
            index --;
        }
        solution[i].cost = solution[i].times.back();
    }
}

bool validateSolution(std::shared_ptr<PlanInstance> instance,
                    const MRTrajectory &solution,
                    double col_dt)
{
    int numRobots = instance->getNumberOfRobots();
    double maxtime = 0;
    for (int i = 0; i < numRobots; i++) {
        maxtime = std::max(maxtime, solution[i].times.back());
    }

    int numSteps = maxtime / col_dt;
    
    std::vector<int> index(numRobots, 0);
    for (int s = 0; s <= numSteps; s++) {
        std::vector<RobotPose> poses;
        double t = s * col_dt;
        for (int i = 0; i < numRobots; i++) {
            while (index[i] + 1 < solution[i].times.size() && solution[i].times[index[i] + 1] <= t) {
                index[i]++;
            }
            if (index[i] + 1 == solution[i].times.size()) {
                // assuming obstacle stays at the end of the trajectory
                poses.push_back(solution[i].trajectory[index[i]]);
            } else {
                double alpha = (t - solution[i].times[index[i]]) / (solution[i].times[index[i] + 1] - solution[i].times[index[i]]);
                RobotPose pose_i = instance->interpolate(solution[i].trajectory[index[i]], solution[i].trajectory[index[i] + 1], alpha);
                poses.push_back(pose_i);
            }
        }
        // check collision
        bool hasCollision = instance->checkCollision(poses, false);
        if (hasCollision) {
            log("Plan has collision at step " + std::to_string(s) +
                 " / " + std::to_string(numSteps), LogLevel::WARN);
            instance->checkCollision(poses, false, true);
            for (int i = 0; i < numRobots; i++) {
                log(poses[i], LogLevel::WARN);
            }
            return false;
        }
    }
    return true;

}

bool validateSolution(std::shared_ptr<PlanInstance> instance,
                       const MRTrajectory &solution) 
{
    int numRobots = instance->getNumberOfRobots();
    int max_count = 0;
    for (int i = 0; i < numRobots; i++) {
        max_count = std::max(max_count, (int)solution[i].trajectory.size());
    }
    
    for (int s = 0; s < max_count; s++) {
        std::vector<RobotPose> poses;
        for (int i = 0; i < numRobots; i++) {
            if (s < solution[i].trajectory.size()) {
                poses.push_back(solution[i].trajectory[s]);
            }
            else {
                poses.push_back(solution[i].trajectory.back());
            }
        }
        bool hasCollision = instance->checkCollision(poses, false);
        if (hasCollision) {
            log("Plan has collision at step " + std::to_string(s) +
                 " / " + std::to_string(max_count), LogLevel::WARN);
            instance->checkCollision(poses, false, true);
            for (int i = 0; i < numRobots; i++) {
                log(poses[i], LogLevel::WARN);
            }
            return false;
        }
    }
    return true;
}

bool validateSolution(std::shared_ptr<PlanInstance> instance,
                     const moveit_msgs::RobotTrajectory &plan_traj)
{
    int numRobots = instance->getNumberOfRobots();
    for (int s = 0; s < plan_traj.joint_trajectory.points.size(); s++) {
        std::vector<RobotPose> poses;
        int dof_s = 0;
        for (int i = 0; i < numRobots; i++) {
            RobotPose pose_i = instance->initRobotPose(i);
            for (int d = 0; d < instance->getRobotDOF(i); d++) {
                pose_i.joint_values[d] = plan_traj.joint_trajectory.points[s].positions[dof_s + d];
            }
            poses.push_back(pose_i);
            dof_s += instance->getRobotDOF(i);
        }
        bool hasCollision = instance->checkCollision(poses, false);
        if (hasCollision) {
            log("Plan has collision at step " + std::to_string(s) +
                 " / " + std::to_string(plan_traj.joint_trajectory.points.size()), LogLevel::WARN);
            instance->checkCollision(poses, false, true);
            for (int i = 0; i < numRobots; i++) {
                log(poses[i], LogLevel::WARN);
            }
            return false;
        }
    }
    return true;
}

bool optimizeTrajectory(std::shared_ptr<PlanInstance> instance,
                        const moveit_msgs::RobotTrajectory& input_trajectory,
                        const std::string& group_name,
                        robot_model::RobotModelConstPtr robot_model,
                        const ros::NodeHandle& node_handle,
                        moveit_msgs::RobotTrajectory& smoothed_traj
                        )
{
    // Create a planning pipeline instance
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

    // Set up the planning request
    planning_interface::MotionPlanRequest req;
    req.group_name = group_name;
    req.start_state.is_diff = false;
    req.start_state.joint_state.name = input_trajectory.joint_trajectory.joint_names;
    req.start_state.joint_state.position = input_trajectory.joint_trajectory.points.front().positions;
    req.start_state.joint_state.velocity = input_trajectory.joint_trajectory.points.front().velocities;
    req.start_state.joint_state.effort.resize(input_trajectory.joint_trajectory.joint_names.size(), 0.0);
    req.planner_id = "RRTstar";

    // add attached object
    for (int robot_id = 0; robot_id < instance->getNumberOfRobots(); robot_id++) {
        std::vector<Object> attached_objs = instance->getAttachedObjects(robot_id);
        for (auto &obj : attached_objs) {
            moveit_msgs::AttachedCollisionObject co;
            co.link_name = obj.parent_link;
            co.object.id = obj.name;
            co.object.header.frame_id = obj.parent_link;
            co.object.operation = co.object.ADD;
            req.start_state.attached_collision_objects.push_back(co);
        }
    }

    // set the goal state
    robot_state::RobotState goal_state(robot_model);
    const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group_name);
    goal_state.setJointGroupPositions(joint_model_group, input_trajectory.joint_trajectory.points.back().positions);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, 0.01, 0.01);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Set the initial trajectory as a path constraint
    req.reference_trajectories.resize(1);
    req.reference_trajectories[0].joint_trajectory.push_back(input_trajectory.joint_trajectory);

    // Set up the planning context
    planning_interface::MotionPlanResponse res;
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // Run the optimization
    bool success = planning_pipeline->generatePlan(planning_scene, req, res);

    if (success)
    {
        // The optimized trajectory is now in res.trajectory_
        res.trajectory_->getRobotTrajectoryMsg(smoothed_traj); 
        smoothed_traj.joint_trajectory.points[0].time_from_start = ros::Duration(0);
        std::vector<RobotPose> last_poses(instance->getNumberOfRobots());
        for (int robot_id = 0; robot_id < instance->getNumberOfRobots(); robot_id++) {
            RobotPose pose = instance->initRobotPose(robot_id);
            for (int d = 0; d < instance->getRobotDOF(robot_id); d++) {
                pose.joint_values[d] = smoothed_traj.joint_trajectory.points[0].positions[d];
            }
            last_poses[robot_id] = pose;
        }
        for (int i = 1; i < smoothed_traj.joint_trajectory.points.size(); i++) {
            int dof_s = 0;
            double max_time = 0;
            for (int robot_id = 0; robot_id < instance->getNumberOfRobots(); robot_id++) {
                RobotPose pose = instance->initRobotPose(robot_id);
                for (int d = 0; d < instance->getRobotDOF(robot_id); d++) {
                    pose.joint_values[d] = smoothed_traj.joint_trajectory.points[i].positions[dof_s + d];
                }

                double dt = instance->computeDistance(pose, last_poses[robot_id]) / instance->getVMax(robot_id);
                max_time = std::max(max_time, dt);

                dof_s += instance->getRobotDOF(robot_id);
                last_poses[robot_id] = pose;
            }
            smoothed_traj.joint_trajectory.points[i].time_from_start = ros::Duration(max_time) + smoothed_traj.joint_trajectory.points[i-1].time_from_start;
        }
        ROS_INFO("Optimized trajectory successfully");
    }
    else
    {
        ROS_ERROR("Failed to optimize trajectory");
    }
    return success;
}

SmoothnessMetrics calculate_smoothness(const MRTrajectory &synced_plan, std::shared_ptr<PlanInstance> instance) {
    int num_robots = instance->getNumberOfRobots();
    double total_squared_jerk = 0;
    double total_directional_consistency = 0;
    int segment_count = 0;

    for (int i = 0; i < num_robots; i++) {
        const auto &trajectory = synced_plan[i].trajectory;
        const auto &times = synced_plan[i].times;
        int dof = instance->getRobotDOF(i);

        for (int k = 0; k < dof; k++) {
            double robot_squared_jerk = 0;
            double robot_duration = times.back() - times.front();
            double robot_distance = trajectory.back().joint_values[k] - trajectory.front().joint_values[k];
            
            for (size_t j = 2; j < trajectory.size() - 1; j++) {
                double dt1 = times[j] - times[j - 1];
                double dt2 = times[j + 1] - times[j];

                if (dt1 > 1e-5 && dt2 > 1e-5) {
                    // Calculate velocity
                    double v1 = (trajectory[j].joint_values[k] - trajectory[j - 1].joint_values[k]) / dt1;
                    double v2 = (trajectory[j + 1].joint_values[k] - trajectory[j].joint_values[k]) / dt2;

                    // Calculate acceleration
                    double a1 = (v1 - (trajectory[j - 1].joint_values[k] - trajectory[j - 2].joint_values[k]) / (times[j - 1] - times[j - 2])) / dt1;
                    double a2 = (v2 - v1) / dt2;

                    // Calculate jerk
                    double jerk = (a2 - a1) / ((dt1 + dt2) / 2);
                    robot_squared_jerk += jerk * jerk * dt2;
                }
            }
            
            segment_count++;            
            if (robot_duration > 0 && robot_distance > 0) {
                total_squared_jerk += sqrt(robot_squared_jerk /(std::pow(robot_duration, 5) * std::pow(robot_distance, 2)));
            }
        }

        // Calculate directional consistency for the robot
        double robot_directional_consistency = 0;
        int robot_segment_count = 0;
        for (size_t j = 1; j < trajectory.size() - 1; j++) {
            double dt1 = times[j] - times[j - 1];
            double dt2 = times[j + 1] - times[j];

            if (dt1 > 1e-5 && dt2 > 1e-5) {
                double dot_product = 0;
                double norm_v1 = 0;
                double norm_v2 = 0;

                for (int k = 0; k < dof; k++) {
                    // Calculate velocity
                    double v1 = (trajectory[j].joint_values[k] - trajectory[j - 1].joint_values[k]) / dt1;
                    double v2 = (trajectory[j + 1].joint_values[k] - trajectory[j].joint_values[k]) / dt2;

                    dot_product += v1 * v2;
                    norm_v1 += v1 * v1;
                    norm_v2 += v2 * v2;
                }

                if (norm_v1 < 1e-5 || norm_v2 < 1e-5) {
                    continue;
                }
                norm_v1 = std::sqrt(norm_v1);
                norm_v2 = std::sqrt(norm_v2);
                double cos_theta = dot_product / (norm_v1 * norm_v2);
                robot_directional_consistency += 1 - cos_theta;
                robot_segment_count++;
            }
        }

        if (robot_segment_count > 0) {
            total_directional_consistency += robot_directional_consistency / robot_segment_count;
        }
    }

    SmoothnessMetrics metrics;
    metrics.normalized_jerk_score = total_squared_jerk / segment_count;
    metrics.directional_consistency = total_directional_consistency / num_robots;

    return metrics;
}
