/**
 * @file lego_policy.h
 * @brief LEGO assembly manipulation policies with force control
 * 
 * This file implements high-level manipulation primitives specifically designed for
 * LEGO assembly operations. It provides force-controlled picking, placing, and support
 * operations that integrate with force/torque sensing for robust manipulation.
 * 
 * Key capabilities:
 * - Force-controlled manipulation primitives (pick, place, support, handover, placeup)
 * - Integration with MFI robot hardware interfaces
 * 
 * @author Philip Huang
 */

#ifndef APEX_MR_LEGO_POLICY_H
#define APEX_MR_LEGO_POLICY_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Trigger.h>
#include "policy.h"
#include "lego/Lego.hpp"
#include "control_msgs/FollowJointTrajectoryAction.h"

#ifdef HAVE_YK_TASKS
#include "yk_tasks/GoToJointsAction.h"
#include "yk_tasks/GoToPoseAction.h"
#include "yk_msgs/GetPose.h"
#include "industrial_msgs/RobotStatus.h"
#endif

struct LegoPolicyCfg {
    double twist_rad = 0.244346;
    double twist_rad_handover = 0.314159;
    double z_force_threshold = -15; // for pick and drop
    double z_force_thresh_w_sup = -5; // for drop with support
    double x_force_threshold = 15; // for place up
    double handover_force_threshold = -10; // for handover
    double dt = 0.1;
    double velocity = 1.0;
    double sup_force_tol = 0.03;
    std::string exec_stats_file = ""; // file to write the time taken for each pick/place/support/handover/place?
};

class LegoPolicy : public Policy {
public:
    LegoPolicy(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr, 
                const std::vector<std::string> & group_names,
                const std::vector<std::vector<std::string>> &joint_names,
                std::shared_ptr<ActivityGraph> act_graph,
                const LegoPolicyCfg &config);
    virtual bool execute(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        Activity::Type type) override;
    virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id) override;

//    virtual void add_actionlib(std::vector<std::shared_ptr<control_msgs::FollowJointTrajectoryActionClient>> action_clients);
    virtual void add_move_clients(const std::vector<ros::ServiceClient> &clients);


#ifdef HAVE_YK_TASKS
    virtual void add_actionlib(const std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>> &action_clients);
    virtual void add_goto_pose_actionlib(const std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToPoseAction>>> &action_clients);
    virtual void add_stop_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &stop_clients);
    virtual void add_enable_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &enable_clients);
    virtual void add_getpose_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &getpose_clients);
#endif

private:
    void save_exec_time(Activity::Type type, const std::shared_ptr<tpg::Node> &start_node, double time);

    bool pickplace(const std::shared_ptr<tpg::Node> &start_node,
                    const std::shared_ptr<tpg::Node> &end_node,
                    Activity::Type type);
    bool support(const std::shared_ptr<tpg::Node> &start_node,
                    const std::shared_ptr<tpg::Node> &end_node);
    bool placeup(const std::shared_ptr<tpg::Node> &start_node,
                    const std::shared_ptr<tpg::Node> &end_node);
    bool pressdown(const std::shared_ptr<tpg::Node> &start_node,
                    const std::shared_ptr<tpg::Node> &end_node);
    std::vector<double> convertQ(const lego_manipulation::math::VectorJd &q_deg);
    void setTrajectory(int robot_id, const std::vector<double>&q1, double t0,
        const std::vector<double> &q2, trajectory_msgs::JointTrajectory &joint_traj);
    void computeVelAcc(trajectory_msgs::JointTrajectory &joint_traj);

    void wrenchCallbackA(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void wrenchCallbackB(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    bool move(int robot_id, const lego_manipulation::math::VectorJd &q_deg);
    bool move(int robot_id, trajectory_msgs::JointTrajectory &joint_traj);
    bool checkForce(int robot_id, Activity::Type task_type, double &force_reading);
    bool stop(int robot_id);
    bool enable(int robot_id);

    std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
    std::vector<std::vector<std::string>> joint_names_;
    std::vector<ros::ServiceClient> clients_;

    ros::NodeHandle nh;
    ros::Subscriber wrench_sub_a_, wrench_sub_b_;
    ros::Subscriber joint_sub_;
    geometry_msgs::WrenchStamped wrench_a_, wrench_b_;
    std::shared_ptr<ActivityGraph> act_graph_;
    LegoPolicyCfg config_;
    std::vector<double> r1_joints_;
    std::vector<double> r2_joints_;
    double x_force_base_ = 0.0;
    double z_force_base_ = 0.0;
    double ts_ = 0.0;
    bool IK_status_;
    bool sup_req_ = false;

    // bool move_actionlib_motoros(int robot_id, const lego_manipulation::math::VectorJd &q_deg, Activity::Type task_type);
    // bool move_actionlib_motoros(int robot_id, const std::vector<double> &q, Activity::Type task_type);
    // std::vector<std::shared_ptr<control_msgs::FollowJointTrajectoryActionClient>> action_clients_motoros_;

#ifdef HAVE_YK_TASKS
    void statusCallbackA(const industrial_msgs::RobotStatus::ConstPtr &msg);
    void statusCallbackB(const industrial_msgs::RobotStatus::ConstPtr &msg);
    bool move_actionlib(int robot_id, const lego_manipulation::math::VectorJd &q_deg, Activity::Type task_type);
    bool move_actionlib(int robot_id, const std::vector<double> &q, Activity::Type task_type);
    //bool move_actionlib(int robot_id, const yk_tasks::GoToPoseGoal &goal, Activity::Type task_type);
    double waitUntilInMotion(int robot_id);
    double waitUntilStopped(int robot_id);
    std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>> action_clients_;
    std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToPoseAction>>> goto_pose_clients_;
    std::vector<std::shared_ptr<ros::ServiceClient>> getpose_clients_;
    std::vector<std::shared_ptr<ros::ServiceClient>> stop_clients_;
    std::vector<std::shared_ptr<ros::ServiceClient>> enable_clients_;
    ros::Subscriber status_sub_a_, status_sub_b_;
    bool moving_a, moving_b;
#endif

    
};

#endif // APEX_MR_LEGO_POLICY_H
