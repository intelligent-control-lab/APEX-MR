#include "lego_policy.h"
#include "lego/Utils/Math.hpp"
#include "logger.h"

LegoPolicy::LegoPolicy(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr, 
                const std::vector<std::string> &group_names,
                const std::vector<std::vector<std::string>> &joint_names,
                std::shared_ptr<ActivityGraph> act_graph,
                const LegoPolicyCfg &config)
    : Policy(), lego_ptr_(lego_ptr), joint_names_(joint_names), act_graph_(act_graph), config_(config)
{
    std::string r1_name = group_names[0];
    std::string r2_name = group_names[1];
    wrench_sub_a_ = nh.subscribe("/yk_destroyer/fts", 1, &LegoPolicy::wrenchCallbackA, this);
    wrench_sub_b_ = nh.subscribe("/yk_architect/fts", 1, &LegoPolicy::wrenchCallbackB, this);
    r1_joints_ = std::vector<double>(6, 0);
    r2_joints_ = std::vector<double>(6, 0);

#ifdef HAVE_YK_TASKS
    action_clients_ = std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>>();
    action_clients_.reserve(2);
    stop_clients_ = std::vector<std::shared_ptr<ros::ServiceClient>>();
    stop_clients_.reserve(2);
    getpose_clients_ = std::vector<std::shared_ptr<ros::ServiceClient>>();
    getpose_clients_.reserve(2);
    goto_pose_clients_ = std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToPoseAction>>>();
    goto_pose_clients_.reserve(2);
    enable_clients_ = std::vector<std::shared_ptr<ros::ServiceClient>>();
    enable_clients_.reserve(2);
    status_sub_a_ = nh.subscribe("/" + r1_name + "/robot_status", 1, &LegoPolicy::statusCallbackA, this);
    status_sub_b_ = nh.subscribe("/" + r2_name + "/robot_status", 1, &LegoPolicy::statusCallbackB, this);
#endif

    if (!config_.exec_stats_file.empty()) {
        std::ofstream ofs(config_.exec_stats_file, std::ios::out);
        ofs << "Task,RobotId,ActId,Duration" << std::endl;
        ofs.close();
    }

}

bool LegoPolicy::execute(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        Activity::Type type)
{
#ifdef HAVE_YK_TASKS
    waitUntilStopped(start_node->robotId);
    ros::Duration(0.5).sleep();
#endif

    auto tic = std::chrono::high_resolution_clock::now();
    bool ret = false;
    if (type == Activity::Type::pick_down || type == Activity::Type::drop_down || type == Activity::Type::handover_down) {
        ret = pickplace(start_node, end_node, type);
    }
    else if (type == Activity::Type::support) {
        ret = support(start_node, end_node);
    }
    else if (type == Activity::Type::place_up) {
        ret = placeup(start_node, end_node);
    }
    else if (type == Activity::Type::press_down) {
        ret = pressdown(start_node, end_node);
    }
    else {
        return false;
    }

    auto toc = std::chrono::high_resolution_clock::now();
    double duration_s = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
    save_exec_time(type, start_node, duration_s);

    return ret;
}

void LegoPolicy::save_exec_time(Activity::Type type, const tpg::NodePtr &node, double duration)
{
    if (!config_.exec_stats_file.empty()) {
        int actId = node->actId;
        int robotId = node->robotId;
        auto act = act_graph_->get(robotId, actId);
        

        std::ofstream ofs(config_.exec_stats_file, std::ios::app);
        ofs << act->type_string() << "," << robotId << "," << actId << "," << duration << std::endl;
        ofs.close();
    }
}

bool LegoPolicy::support(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node)
{
    int robot_id = start_node->robotId;
    RobotPose start_pose = start_node->pose;
    RobotPose end_pose = end_node->pose;
    bool success = true;

    Eigen::MatrixXd sup_T;
    lego_manipulation::math::VectorJd sup_q;
    if (robot_id == 0) {
        sup_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
        sup_q.col(0) << end_pose.joint_values[0], end_pose.joint_values[1], end_pose.joint_values[2], end_pose.joint_values[3], end_pose.joint_values[4], end_pose.joint_values[5];
        sup_q = sup_q / M_PI * 180;
        sup_T = lego_manipulation::math::FK(sup_q, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), false);
    }
    else {
        sup_q = Eigen::MatrixXd(lego_ptr_->robot_dof_2(), 1);
        sup_q.col(0) << end_pose.joint_values[0], end_pose.joint_values[1], end_pose.joint_values[2], end_pose.joint_values[3], end_pose.joint_values[4], end_pose.joint_values[5];
        sup_q = sup_q / M_PI * 180;
        sup_T = lego_manipulation::math::FK(sup_q, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), false);
    }

    lego_manipulation::math::VectorJd init_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
    init_q.col(0) << 0, -15.456, -40.3566, 0, 30, 0;

    // Eigen::MatrixXd delta_down_T = Eigen::MatrixXd::Identity(4, 4);
    // delta_down_T.col(3) << -0.001, 0, 0, 1;
    Eigen::MatrixXd delta_up_T = Eigen::MatrixXd::Identity(4, 4);
    // delta_up_T.col(3) << 0.0005, 0, 0, 1;
    
    // std::cout << sup_T << std::endl;
    // sup_T = sup_T * delta_down_T;
    // std::cout << sup_T << std::endl;
    // // ik
    // if (robot_id == 0) {
    //     sup_q = lego_manipulation::math::IK(sup_q, sup_T.block(0, 3, 3, 1), sup_T.block(0, 0, 3, 3),
    //                             lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
    // }
    // else {
    //     sup_q = lego_manipulation::math::IK(init_q, sup_T.block(0, 3, 3, 1), sup_T.block(0, 0, 3, 3),
    //                             lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
    // }

    log("Robot " + std::to_string(robot_id) + " supporting ...", LogLevel::INFO);

#ifdef HAVE_YK_TASKS

    // // send the sup_T to the robot
    // move_actionlib(robot_id, sup_q, Activity::Type::support_pre);
    // waitUntilStopped(robot_id);
    // ros::Duration(0.5).sleep();


    delta_up_T.col(3) << 0.05, 0, 0, 1;
    sup_T = sup_T * delta_up_T;
    std::cout << sup_T << std::endl;
    lego_manipulation::math::VectorJd sup_up_q;
    if (robot_id == 0) {
        sup_up_q = lego_manipulation::math::IK(sup_q, sup_T.block(0, 3, 3, 1), sup_T.block(0, 0, 3, 3),
                                lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
    }
    else {
        sup_up_q = lego_manipulation::math::IK(sup_q, sup_T.block(0, 3, 3, 1), sup_T.block(0, 0, 3, 3),
                                lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
    }
    move_actionlib(robot_id, sup_up_q, Activity::Type::support);
    waitUntilStopped(robot_id);
    ros::Duration(0.5).sleep();

    // update support q
    std::vector<double> q_cur = (robot_id == 0) ? r1_joints_ : r2_joints_;
    sup_up_q.col(0) << q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4], q_cur[5];
    sup_up_q = sup_up_q / M_PI * 180;     

    log("Robot " + std::to_string(robot_id) + " supported", LogLevel::INFO);
#endif
    return success;

}

bool LegoPolicy::pressdown(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node)
{
    int robot_id = start_node->robotId;
    log("Robot ID: " + std::to_string(robot_id) + " pressing down for support ...", LogLevel::INFO);
    RobotPose start_pose = start_node->pose;
    RobotPose end_pose = end_node->pose;
    bool success = true;

    Eigen::MatrixXd press_up_T, press_T;
    lego_manipulation::math::VectorJd press_up_q;
    if (robot_id == 0) {
        press_up_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
        press_up_q.col(0) << start_pose.joint_values[0], start_pose.joint_values[1], start_pose.joint_values[2],
                     start_pose.joint_values[3], start_pose.joint_values[4], start_pose.joint_values[5];
        press_up_q = press_up_q / M_PI * 180;
        press_up_T = lego_manipulation::math::FK(press_up_q, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), false);
    }
    else {
        press_up_q = Eigen::MatrixXd(lego_ptr_->robot_dof_2(), 1);
        press_up_q.col(0) << start_pose.joint_values[0], start_pose.joint_values[1], start_pose.joint_values[2],
                     start_pose.joint_values[3], start_pose.joint_values[4], start_pose.joint_values[5];
        press_up_q = press_up_q / M_PI * 180;
        press_up_T = lego_manipulation::math::FK(press_up_q, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), false);
    }

    lego_manipulation::math::VectorJd press_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);

    Eigen::MatrixXd delta_down_T = Eigen::MatrixXd::Identity(4, 4);
    delta_down_T.col(3) << 0, 0, 0.05, 1;
    
    press_T = press_up_T * delta_down_T;
    
    // ik
    if (robot_id == 0) {
        press_q = lego_ptr_->IK(press_up_q, press_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(),
                        lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_);
        if (!IK_status_) {
            log("Robot 1 IK failed", LogLevel::ERROR);
            return false;
        }
    }
    else {
        press_q = lego_ptr_->IK(press_up_q, press_T, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(),
                        lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_inv_r2(), 0, IK_status_);
        if (!IK_status_) {
            log("Robot 2 IK failed", LogLevel::ERROR);
            return false;
        }
    }

#ifdef HAVE_YK_TASKS

    // send the sup_T to the robot
    move_actionlib(robot_id, press_q, Activity::Type::press_down);
    waitUntilStopped(robot_id);
    ros::Duration(0.5).sleep();
    log("Robot " + std::to_string(robot_id) + " pressed down (support)", LogLevel::INFO);

#endif

    return success;

}

bool LegoPolicy::pickplace(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        Activity::Type type)
{
    int robot_id = start_node->robotId;
    RobotPose start_pose = start_node->pose;
    RobotPose end_pose = end_node->pose;
    bool success = true;

    // first start with the current pose
    Eigen::MatrixXd up_T;
    Eigen::MatrixXd cart_T;
    lego_manipulation::math::VectorJd up_q;
    log(start_pose, LogLevel::INFO);
    if (robot_id == 0) {
        up_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
        up_q.col(0) << start_pose.joint_values[0], start_pose.joint_values[1], start_pose.joint_values[2], start_pose.joint_values[3], start_pose.joint_values[4], start_pose.joint_values[5];
        up_q = up_q / M_PI * 180;
        up_T = lego_manipulation::math::FK(up_q, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), false);
    }
    else {
        up_q = Eigen::MatrixXd(lego_ptr_->robot_dof_2(), 1);
        up_q.col(0) << start_pose.joint_values[0], start_pose.joint_values[1], start_pose.joint_values[2], start_pose.joint_values[3], start_pose.joint_values[4], start_pose.joint_values[5];
        up_q = up_q / M_PI * 180;
        up_T = lego_manipulation::math::FK(up_q, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), false);
    }
    cart_T = up_T;
    
    // keep pressing down the object
    Eigen::MatrixXd delta_down_T = Eigen::MatrixXd::Identity(4, 4);
    delta_down_T.col(3) << 0, 0, 0.0005, 1;
    Eigen::MatrixXd delta_up_T = Eigen::MatrixXd::Identity(4, 4);
    delta_up_T.col(3) << 0, 0, -0.0005, 1;

    if (type == Activity::Type::drop_down) {
        int actId = start_node->actId;
        auto act = act_graph_->get(robot_id, actId);
        for (auto &act_prev : act->type2_prev) {
            if (act_prev->type == Activity::Type::support) {
                sup_req_ = true;
            }
        }
    }

    bool pressed = false;
    log("Robot " + std::to_string(robot_id) + " pressing down...", LogLevel::INFO);
    lego_manipulation::math::VectorJd down_q;

#ifdef HAVE_YK_TASKS
    if (type == Activity::Type::pick_down) {
        delta_down_T.col(3) << 0, 0, 0.05, 1;
    }
    else {
        delta_down_T.col(3) << 0, 0, 0.05, 1;
    }
    cart_T = cart_T * delta_down_T;
    if (robot_id == 0) {
        down_q = lego_manipulation::math::IK(up_q, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
    }
    else {
        down_q = lego_manipulation::math::IK(up_q, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
    }
    move_actionlib(robot_id, down_q, type);
    ros::Duration(0.5).sleep();

    // set the delta_down_T to how much the robot actually moved down
    std::vector<double> q_cur = (robot_id == 0) ? r1_joints_ : r2_joints_;
    down_q.col(0) << q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4], q_cur[5];
    down_q = down_q / M_PI * 180;
#endif
    sup_req_ = false;

    // then twist to pick or place
    log("Robot " + std::to_string(robot_id) + " pressed, twisting...", LogLevel::INFO);
    Eigen::MatrixXd twist_R_pick = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd twist_R_place = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd twist_R_handover = Eigen::MatrixXd::Identity(3, 3);
    twist_R_pick << cos(config_.twist_rad), 0, sin(config_.twist_rad), 
                0, 1, 0, 
                -sin(config_.twist_rad), 0, cos(config_.twist_rad);
    twist_R_place << cos(-config_.twist_rad), 0, sin(-config_.twist_rad), 
                0, 1, 0, 
                -sin(-config_.twist_rad), 0, cos(-config_.twist_rad);
    twist_R_handover << cos(-config_.twist_rad_handover), 0, sin(-config_.twist_rad_handover), 
                0, 1, 0, 
                -sin(-config_.twist_rad_handover), 0, cos(-config_.twist_rad_handover);
    Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
    if (type == Activity::Type::pick_down) {
        twist_T.block(0, 0, 3, 3) << twist_R_pick;
        if (robot_id == 0) {
            cart_T = lego_manipulation::math::FK(down_q, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(down_q, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        twist_T = cart_T * twist_T;
    }
    else if (type == Activity::Type::drop_down) {
        twist_T.block(0, 0, 3, 3) << twist_R_place;
        if (robot_id == 0) {
            cart_T = lego_manipulation::math::FK(down_q, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(down_q, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        twist_T = cart_T * twist_T;
    }
    else if (type == Activity::Type::handover_down) {
        twist_T.block(0, 0, 3, 3) << twist_R_handover;
        if (robot_id == 0) {
            cart_T = lego_manipulation::math::FK(down_q, lego_ptr_->robot_DH_tool_handover_assemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(down_q, lego_ptr_->robot_DH_tool_handover_assemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        twist_T = cart_T * twist_T;

    }
    lego_manipulation::math::VectorJd twist_q;
    if (robot_id == 0) {
        if (type == Activity::Type::pick_down) {
            twist_q = lego_manipulation::math::IK(down_q, twist_T.block(0, 3, 3, 1), twist_T.block(0, 0, 3, 3),
                                    lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
        }
        else if (type == Activity::Type::drop_down) { 
            twist_q = lego_manipulation::math::IK(down_q, twist_T.block(0, 3, 3, 1), twist_T.block(0, 0, 3, 3),
                                    lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
        }
        else { // handover_down
            twist_q = lego_manipulation::math::IK(down_q, twist_T.block(0, 3, 3, 1), twist_T.block(0, 0, 3, 3),
                                    lego_ptr_->robot_DH_tool_handover_assemble_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
        }
    }
    else {
        if (type == Activity::Type::pick_down) {
            twist_q = lego_manipulation::math::IK(down_q, twist_T.block(0, 3, 3, 1), twist_T.block(0, 0, 3, 3),
                                    lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
        }
        else if (type == Activity::Type::drop_down) {
            twist_q = lego_manipulation::math::IK(down_q, twist_T.block(0, 3, 3, 1), twist_T.block(0, 0, 3, 3),
                                    lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
        }
        else { // handover_down
            twist_q = lego_manipulation::math::IK(down_q, twist_T.block(0, 3, 3, 1), twist_T.block(0, 0, 3, 3),
                                    lego_ptr_->robot_DH_tool_handover_assemble_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
        }
    };
    std::vector<double> twist_up_q(end_pose.joint_values.begin(), end_pose.joint_values.begin()+6);

#if HAVE_YK_TASKS
    move_actionlib(robot_id, twist_q, Activity::Type::pick_twist);
    waitUntilStopped(robot_id);
    ros::Duration(0.5).sleep();
    move_actionlib(robot_id, twist_up_q, Activity::Type::pick_twist_up);
    waitUntilStopped(robot_id);
    ros::Duration(0.5).sleep();    
#endif

    return success;
}

bool LegoPolicy::placeup(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node)
{
    int robot_id = start_node->robotId;
    RobotPose start_pose = start_node->pose;
    RobotPose end_pose = end_node->pose;
    bool success = true;

    // first start with the current pose
    Eigen::MatrixXd down_T;
    Eigen::MatrixXd cart_T;
    lego_manipulation::math::VectorJd down_q;
    log(start_pose, LogLevel::INFO);
    if (robot_id == 0) {
        down_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
        down_q.col(0) << start_pose.joint_values[0], start_pose.joint_values[1], start_pose.joint_values[2], start_pose.joint_values[3], start_pose.joint_values[4], start_pose.joint_values[5];
        down_q = down_q / M_PI * 180;
        down_T = lego_manipulation::math::FK(down_q, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), false);
    }
    else {
        down_q = Eigen::MatrixXd(lego_ptr_->robot_dof_2(), 1);
        down_q.col(0) << start_pose.joint_values[0], start_pose.joint_values[1], start_pose.joint_values[2], start_pose.joint_values[3], start_pose.joint_values[4], start_pose.joint_values[5];
        down_q = down_q / M_PI * 180;
        down_T = lego_manipulation::math::FK(down_q, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), false);
    }
    cart_T = down_T;
    
    // keep pressing up the object
    Eigen::MatrixXd delta_up_T = Eigen::MatrixXd::Identity(4, 4);
    delta_up_T.col(3) << -0.05, 0, 0, 1;

    bool pressed = false;
    log("Robot " + std::to_string(robot_id) + " pressing up...", LogLevel::INFO);
    lego_manipulation::math::VectorJd up_q;

#ifdef HAVE_YK_TASKS
    cart_T = cart_T * delta_up_T;
    if (robot_id == 0) {
        up_q = lego_ptr_->IK(down_q, cart_T, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), 
                                        lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_alt_inv_r1(), 0, IK_status_);
        if (!IK_status_) {
            log("Robot 1 IK failed", LogLevel::ERROR);
            return false;
        }
    }
    else {
        up_q = lego_ptr_->IK(down_q, cart_T, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), 
                                        lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_alt_inv_r2(), 0, IK_status_);
        if (!IK_status_) {
            log("Robot 2 IK failed", LogLevel::ERROR);
            return false;
        }
    }
    move_actionlib(robot_id, up_q, Activity::Type::place_up);
    ros::Duration(0.5).sleep();

    // set the delta_up_T to how much the robot actually moved down
    std::vector<double> q_cur = (robot_id == 0) ? r1_joints_ : r2_joints_;
    up_q.col(0) << q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4], q_cur[5];
    up_q = up_q / M_PI * 180;

#endif

    // then twist to pick or place
    log("Robot " + std::to_string(robot_id) + " pressed up, twisting...", LogLevel::INFO);
    Eigen::MatrixXd twist_R_pick = Eigen::MatrixXd::Identity(3, 3);
    twist_R_pick << cos(config_.twist_rad), 0, sin(config_.twist_rad), 
                0, 1, 0, 
                -sin(config_.twist_rad), 0, cos(config_.twist_rad);
    Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
    twist_T.block(0, 0, 3, 3) << twist_R_pick;
    if (robot_id == 0) {
        cart_T = lego_manipulation::math::FK(up_q, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), false);
    }
    else {
        cart_T = lego_manipulation::math::FK(up_q, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), false);
    } 
    twist_T = cart_T * twist_T;
    
    lego_manipulation::math::VectorJd twist_q;
    if (robot_id == 0) {
        twist_q = lego_ptr_->IK(up_q, twist_T, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(),
                        lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_alt_inv_r1(), 0, IK_status_);
        if (!IK_status_) {
            log("Robot 1 twist IK failed", LogLevel::ERROR);
            return false;
        }
    }
    else {
        twist_q = lego_ptr_->IK(up_q, twist_T, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(),
                        lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_alt_inv_r2(), 0, IK_status_);
        if (!IK_status_) {
            log("Robot 2 twist IK failed", LogLevel::ERROR);
            return false;
        }
    }
    std::vector<double> twist_down_q(end_pose.joint_values.begin(), end_pose.joint_values.begin()+6);

#if HAVE_YK_TASKS
    move_actionlib(robot_id, twist_q, Activity::Type::place_twist);
    waitUntilStopped(robot_id);
    ros::Duration(0.5).sleep();
    move_actionlib(robot_id, twist_down_q, Activity::Type::place_twist_down);
    waitUntilStopped(robot_id);
    ros::Duration(0.5).sleep();
#endif

    return success;
}


std::vector<double> LegoPolicy::convertQ(const lego_manipulation::math::VectorJd &q_deg) {
    std::vector<double> q;
    for (int i = 0; i < 6; i++) {
        q.push_back(q_deg(i) * M_PI / 180);
    }
    return q;
}

void LegoPolicy::setTrajectory(int robot_id, const std::vector<double>&q1, double t0,
        const std::vector<double> &q2, trajectory_msgs::JointTrajectory &joint_traj) {
    joint_traj.points.clear();
    
    // move the robot to the target joint state
    joint_traj.joint_names = joint_names_[robot_id];
    int dof = joint_names_[robot_id].size();

    // interpolate the joint states and save as a trajectory
    double distance = 0;
    for (int i = 0; i < dof; i++) {
        distance += std::abs(q2[i] - q1[i]);
    }
    int num_points = std::ceil(distance / (config_.velocity * config_.dt));
    std::cout << "num_points: " << num_points << std::endl;
    
    int j = 0;
    do {
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(j * config_.dt + t0);
        point.positions.resize(dof);
        point.velocities.resize(dof);
        point.accelerations.resize(dof);

        double alpha = j * 1.0 / num_points;
        for (int d = 0; d < dof; d++) {
            point.positions[d] = q1[d] * (1-alpha) + q2[d] * alpha;
        }
        joint_traj.points.push_back(point);
        j++;
    } while (j < num_points);
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(j * config_.dt + t0);
    point.positions.resize(dof);
    point.velocities.resize(dof);
    point.accelerations.resize(dof);
    for (int d = 0; d < dof; d++) {
        point.positions[d] = q2[d];
    }
    joint_traj.points.push_back(point); 

    return;
}

void LegoPolicy::computeVelAcc(trajectory_msgs::JointTrajectory &joint_traj) {
    // compute velocities and accelerations with central difference
    int dof = 6;
    double dt = config_.dt;

    for (int i = 1; i < joint_traj.points.size() - 1; i++) {
        for (int j = 0; j < dof; j++) {
            joint_traj.points[i].velocities[j] = (joint_traj.points[i+1].positions[j] - joint_traj.points[i-1].positions[j]) / (2 * dt);
            joint_traj.points[i].accelerations[j] = (joint_traj.points[i+1].positions[j] - 2 * joint_traj.points[i].positions[j] + joint_traj.points[i-1].positions[j]) / (dt * dt);
        }
    }
}

bool LegoPolicy::move(int robot_id, const lego_manipulation::math::VectorJd &q_deg) {
    std::vector<double> q = convertQ(q_deg);
    std::vector<double> q_cur = (robot_id == 0) ? r1_joints_ : r2_joints_;

    trajectory_msgs::JointTrajectory joint_traj;
    setTrajectory(robot_id, q_cur, 0, q, joint_traj);
    return move(robot_id, joint_traj);
    // yk_msgs::SetJoints srv;
    // srv.request.state.name = joint_names_[robot_id];
    // srv.request.state.position = convertQ(q_deg);
    // bool result = setjoint_clients_[robot_id].call(srv);

    // if (!result) {
    //     log("Failed to call service for robot " + std::to_string(robot_id), LogLevel::ERROR);
    //     return false;
    // }
    // std::cout << "set joint clients_ call result: " << std::endl;

    // return true;
}

bool LegoPolicy::move(int robot_id, trajectory_msgs::JointTrajectory &joint_traj)
{
    auto q = joint_traj.points.back().positions;
    log("Robot " + std::to_string(robot_id) + " moving to joint state " + 
        std::to_string(q[0]) + " " + std::to_string(q[1]) + " " + std::to_string(q[2]) + " " +
        std::to_string(q[3]) + " " + std::to_string(q[4]) + " " + std::to_string(q[5]), LogLevel::INFO);
    moveit_msgs::ExecuteKnownTrajectory srv;
    srv.request.wait_for_execution = true;
    srv.request.trajectory.joint_trajectory = joint_traj;
    
    computeVelAcc(joint_traj);
    
    std::vector<double> q_cur = (robot_id == 0) ? r1_joints_ : r2_joints_;
    log("Robot " + std::to_string(robot_id) + " current state " + 
        std::to_string(q_cur[0]) + " " + std::to_string(q_cur[1]) + " " + std::to_string(q_cur[2]) + " " +
        std::to_string(q_cur[3]) + " " + std::to_string(q_cur[4]) + " " + std::to_string(q_cur[5]), LogLevel::INFO);
    joint_traj.points.front().positions = q_cur;

    bool result = clients_[robot_id].call(srv);
         
    if (!result) {
        log("Failed to call service for robot " + std::to_string(robot_id), LogLevel::ERROR);
        return false;
    }
        
    int error_code = srv.response.error_code.val;
    log("Robot " + std::to_string(robot_id) + " traj execute service, code " + std::to_string(error_code), LogLevel::DEBUG);
    if (error_code == moveit_msgs::MoveItErrorCodes::TIMED_OUT) {
        log("Execute Timeout, retrying...", LogLevel::WARN);
        ros::Duration(0.01).sleep();
        return move(robot_id, joint_traj);
    }
    else if (error_code < 0) {
        return false;
    }
    else {
        log("Robot " + std::to_string(robot_id) + " success, moving to the next segment", LogLevel::DEBUG);
        return true;
    }

}


void LegoPolicy::add_move_clients(const std::vector<ros::ServiceClient> &clients)
{
    clients_ = clients;
}

// void LegoPolicy::add_actionlib(std::vector<std::shared_ptr<control_msgs::FollowJointTrajectoryActionClient>> action_clients)
// {
//     for (auto client: action_clients) {
//         action_clients_motoros_.push_back(client);
//     }
//     log(std::to_string(action_clients_.size()) + " action clients added", LogLevel::INFO);
// }

// bool LegoPolicy::move_actionlib_motoros(int robot_id, const lego_manipulation::math::VectorJd& q_deg)
// {
//     std::vector<double> q = convertQ(q_deg);
//     return move_actionlib_motoros(robot_id, q);
// }

// bool LegoPolicy::move_actionlib_motoros(int robot_id, const std::vector<double> &q, Activity::Type task_type)
// {
//     control_msgs::FollowJointTrajectoryGoal goal;
//     goal.trajectory.joint_names = joint_names_[robot_id];
//     return false;
// }

#ifdef HAVE_YK_TASKS

void LegoPolicy::add_actionlib(const std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>> &action_clients)
{
    for (auto client: action_clients) {
        action_clients_.push_back(client);
    }
    log(std::to_string(action_clients_.size()) + " action clients added", LogLevel::INFO);
}

void LegoPolicy::add_stop_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &stop_clients)
{
    for (auto client: stop_clients) {
        stop_clients_.push_back(client);
    }
    log(std::to_string(clients_.size()) + " stop clients added", LogLevel::INFO);
}

void LegoPolicy::add_enable_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &enable_clients)
{
    for (auto client: enable_clients) {
        enable_clients_.push_back(client);
    }
    log(std::to_string(enable_clients_.size()) + " enable clients added", LogLevel::INFO);
}

void LegoPolicy::add_getpose_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &getpose_clients)
{
    for (auto client: getpose_clients) {
        getpose_clients_.push_back(client);
    }
    log(std::to_string(getpose_clients_.size()) + " getpose clients added", LogLevel::INFO);
}

void LegoPolicy::add_goto_pose_actionlib(const std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToPoseAction>>> &goto_pose_clients)
{
    for (auto client: goto_pose_clients) {
        goto_pose_clients_.push_back(client);
    }
    log(std::to_string(goto_pose_clients_.size()) + " goto pose clients added", LogLevel::INFO);
}

bool LegoPolicy::move_actionlib(int robot_id, const lego_manipulation::math::VectorJd& q_deg, Activity::Type task_type)
{
    std::vector<double> q = convertQ(q_deg);
    bool res = move_actionlib(robot_id, q, task_type);
    return res;
}    

bool LegoPolicy::move_actionlib(int robot_id, const std::vector<double> &q, Activity::Type task_type)
{
    yk_tasks::GoToJointsGoal goal;
    goal.state.name = joint_names_[robot_id];
    goal.state.position = q;
    goal.state.velocity.resize(q.size());
    goal.state.effort.resize(q.size());
    if (task_type == Activity::Type::support || task_type == Activity::Type::press_down) {
        goal.max_velocity_scaling_factor = 0.0002;
        goal.max_acceleration_scaling_factor = 0.2;   
    }
    else if (task_type == Activity::Type::pick_down || task_type == Activity::Type::drop_down ||
            task_type == Activity::Type::handover_down || task_type == Activity::Type::place_up) {     
        goal.max_velocity_scaling_factor = 0.001;
        goal.max_acceleration_scaling_factor = 0.2;
    }
    else {
        goal.max_velocity_scaling_factor = 0.02;
        goal.max_acceleration_scaling_factor = 0.2;
    }
    log("Robot" + std::to_string(robot_id) + " moving using action client", LogLevel::INFO);

    if (action_clients_.size() < robot_id) {
        log("Action client not initialized for robot " + std::to_string(robot_id), LogLevel::ERROR);
        return false;
    }
    action_clients_[robot_id]->sendGoal(goal);

    if (task_type == Activity::Type::support || task_type == Activity::Type::pick_down || task_type == Activity::Type::drop_down
        || task_type == Activity::Type::handover_down || task_type == Activity::Type::place_up || task_type == Activity::Type::press_down) {     
            
        log("Start Waiting until in motion", LogLevel::INFO);
        double waited = waitUntilInMotion(robot_id);
        log("End Waiting until in motion, waited " + std::to_string(waited) + " s", LogLevel::INFO);
        x_force_base_ = (robot_id == 0) ? wrench_a_.wrench.force.x : wrench_b_.wrench.force.x;
        z_force_base_ = (robot_id == 0) ? wrench_a_.wrench.force.z : wrench_b_.wrench.force.z;
        log("Robot " + std::to_string(robot_id) + " x force base " + std::to_string(x_force_base_) + " z force base " + std::to_string(z_force_base_), LogLevel::INFO);
        while (!action_clients_[robot_id]->waitForResult(ros::Duration(0.001)))
        {
            double f_reading;
            bool stop_check = checkForce(robot_id, task_type, f_reading);
            if (stop_check) {
                log("Force exceeded " + std::to_string(f_reading) + "N, stopping the yk robot " + std::to_string(robot_id), LogLevel::INFO);
                double t_goal_cancel = ros::Time::now().toSec();
                //action_clients_[robot_id]->cancelGoal();
                stop(robot_id);

                // run the command line code "rosservice call /yk_destroyer/yk_stop_trajectory "{}" " to stop the robot
                // std::string command = "rosservice call /yk_destroyer/yk_stop_trajectory \"{}\"";
                // int result = system(command.c_str());

                // //***to be deleted***
                // ros::NodeHandle nh;
                // ros::ServiceClient stop_client = nh.serviceClient<std_srvs::Trigger>("yk_destroyer/yk_stop_trajectory", true);
                // std_srvs::Trigger srv;

                // if (!stop_client.call(srv)) 
                // {
                //     log("Failed to call stop service for robot " + std::to_string(robot_id), LogLevel::ERROR);
                //     return false;
                // }
                // //***to be deleted***

                double t_goal_canceled = ros::Time::now().toSec();
                checkForce(robot_id, task_type, f_reading);
                log("Stopped the yk robot " + std::to_string(robot_id) + " force " + std::to_string(f_reading)
                    + " in " + std::to_string(t_goal_canceled - t_goal_cancel) + " s", LogLevel::INFO);
                return true;
            }
        }
        std::cout << "ended move actionlib " << std::endl;
    }
    else {
        return action_clients_[robot_id]->waitForResult(ros::Duration(10.0));
    }
    return false;

}

double LegoPolicy::waitUntilInMotion(int robot_id) {
    ros::Rate r(100);
    double t0 = ros::Time::now().toSec();
    while (ros::ok()) {
        bool moving = (robot_id == 0) ? moving_a : moving_b;
        if (!moving) {
            r.sleep();
        }
        else {
            break;
        }
    }
    double t1 = ros::Time::now().toSec();
    return t1 - t0;
}

double LegoPolicy::waitUntilStopped(int robot_id) {
    ros::Rate r(100);
    double t0 = ros::Time::now().toSec();
    while (ros::ok()) {
        bool moving = (robot_id == 0) ? moving_a : moving_b;
        if (moving) {
            r.sleep();
        }
        else {
            break;
        }
    }
    double t1 = ros::Time::now().toSec();
    return t1 - t0;
}

bool LegoPolicy::stop(int robot_id)
{
    std_srvs::Trigger srv;
    if (!stop_clients_[robot_id]->call(srv)) 
    {
        log("Failed to call stop service for robot " + std::to_string(robot_id), LogLevel::ERROR);
        return false;
    }
    log("stopped robot " + std::to_string(robot_id) + " status " + std::to_string(srv.response.success), LogLevel::INFO);
    waitUntilStopped(robot_id);
    return srv.response.success;
}

bool LegoPolicy::enable(int robot_id)
{
    std_srvs::Trigger srv;
    if (!enable_clients_[robot_id]->call(srv)) 
    {
        log("Failed to call enable service for robot " + std::to_string(robot_id), LogLevel::ERROR);
        return false;
    }
    log("enabled robot " + std::to_string(robot_id) + " status " + std::to_string(srv.response.success), LogLevel::INFO);
    return srv.response.success;
}

void LegoPolicy::statusCallbackA(const industrial_msgs::RobotStatus::ConstPtr &msg)
{
    moving_a = msg->in_motion.val;
}

void LegoPolicy::statusCallbackB(const industrial_msgs::RobotStatus::ConstPtr &msg)
{
    moving_b = msg->in_motion.val;
}


#endif

bool LegoPolicy::checkForce(int robot_id, Activity::Type task_type, double &force_reading)
{
    if (task_type == Activity::Type::pick_down || task_type == Activity::Type::drop_down) {
        double z_force = (robot_id == 0) ? wrench_a_.wrench.force.z : wrench_b_.wrench.force.z;
        log("Robot " + std::to_string(robot_id) + " z force " + std::to_string(z_force), LogLevel::DEBUG);
        double thresh = config_.z_force_threshold;
        if (sup_req_) {
            thresh = config_.z_force_thresh_w_sup;
        }
        if (z_force < thresh) {
            force_reading = z_force;
            return true;
        }
    }
    else if (task_type == Activity::Type::handover_down) {
        double z_force = (robot_id == 0) ? wrench_a_.wrench.force.z : wrench_b_.wrench.force.z;
        log("Robot " + std::to_string(robot_id) + " z force " + std::to_string(z_force), LogLevel::DEBUG);
        if (z_force < config_.handover_force_threshold) {
            force_reading = z_force;
            return true;
        }
    }
    else if (task_type == Activity::Type::place_up) {
        double x_force = (robot_id == 0) ? wrench_a_.wrench.force.x : wrench_b_.wrench.force.x;
        log("Robot " + std::to_string(robot_id) + " x force " + std::to_string(x_force), LogLevel::DEBUG);
        if (x_force > config_.x_force_threshold) {
            force_reading = x_force;
            return true;
        }
    }
    else if (task_type == Activity::Type::support) {
        double x_force = (robot_id == 0) ? wrench_a_.wrench.force.x : wrench_b_.wrench.force.x;
        log("Robot " + std::to_string(robot_id) + " x force " + std::to_string(x_force), LogLevel::DEBUG);
        if (x_force < (x_force_base_ - config_.sup_force_tol)) {
            force_reading = x_force;
            return true;
        }
    }
    else if (task_type == Activity::Type::press_down) {
        double z_force = (robot_id == 0) ? wrench_a_.wrench.force.z : wrench_b_.wrench.force.z;
        log("Robot " + std::to_string(robot_id) + " z force " + std::to_string(z_force), LogLevel::DEBUG);
        if (z_force < z_force_base_ - config_.sup_force_tol) {
            force_reading = z_force;
            return true;
        }
    }
    return false;
}

void LegoPolicy::wrenchCallbackA(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    wrench_a_ = *msg;
}

void LegoPolicy::wrenchCallbackB(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    wrench_b_ = *msg;
}

void LegoPolicy::update_joint_states(const std::vector<double> &joint_states, int robot_id)
{
    if (robot_id == 0) {
        for (int i = 0; i < 6; i++) {
            r1_joints_[i] = joint_states[i];
        }
    }
    else if (robot_id == 1) {
        for (int i = 0; i < 6; i++) {
            r2_joints_[i] = joint_states[i];
        }
        // double tnow = ros::Time::now().toNSec() * 1e-9;
        // std::cout << "updated robot state, time " << (ts_ - tnow) << std::endl;
        // ts_ = tnow;
    }
}
