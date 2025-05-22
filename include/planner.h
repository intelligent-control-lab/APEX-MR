#ifndef APEX_MR_PLANNER_H
#define APEX_MR_PLANNER_H

#include "instance.h" // Include the abstract problem instance definition
#include <memory>
#include <vector>


// utils
bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    MRTrajectory &solution,
                    bool reset_speed = true);


bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    int robot_id,
                    RobotTrajectory &solution);

bool saveSolution(std::shared_ptr<PlanInstance> instance,
                  const moveit_msgs::RobotTrajectory &plan_traj,
                  const std::string &file_name);
                  
bool saveSolution(std::shared_ptr<PlanInstance> instance,
                  const MRTrajectory &synced_traj,
                  const std::string &file_name);

/* time is assumed to be uniform as dt */
bool loadSolution(std::shared_ptr<PlanInstance> instance,
                  const std::string &file_name,
                  double dt,
                  moveit_msgs::RobotTrajectory &plan_traj);

/* time is supplied in the first column*/
bool loadSolution(std::shared_ptr<PlanInstance> instance,
                  const std::string &file_name,
                  moveit_msgs::RobotTrajectory &plan_traj);

bool validateSolution(std::shared_ptr<PlanInstance> instance,
                    const MRTrajectory &solution,
                    double col_dt);

/* assuming uniform discretiziation, check for collisions*/
bool validateSolution(std::shared_ptr<PlanInstance> instance,
                       const MRTrajectory &solution);

void retimeSolution(std::shared_ptr<PlanInstance> instance,
                    const MRTrajectory &solution,
                    MRTrajectory &retime_solution,
                    double dt);

void rediscretizeSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    moveit_msgs::RobotTrajectory &retime_traj,
                    double new_dt);

void rediscretizeSolution(std::shared_ptr<PlanInstance> instance,
                        const MRTrajectory &solution,
                        MRTrajectory &retime_solution,
                        double new_dt);
void removeWait(std::shared_ptr<PlanInstance> instance,
                        MRTrajectory &solution);
bool validateSolution(std::shared_ptr<PlanInstance> instance,
                     const moveit_msgs::RobotTrajectory &plan_traj);

bool optimizeTrajectory(std::shared_ptr<PlanInstance> instance,
                        const moveit_msgs::RobotTrajectory& input_trajectory,
                        const std::string& group_name,
                        robot_model::RobotModelConstPtr robot_model,
                        const ros::NodeHandle& node_handle,
                        moveit_msgs::RobotTrajectory& smoothed_traj
                        );
struct SmoothnessMetrics {
    double normalized_jerk_score;
    double directional_consistency;
};
SmoothnessMetrics calculate_smoothness(const MRTrajectory &synced_plan, std::shared_ptr<PlanInstance> instance);

#endif // APEX_MR_PLANNER_H
