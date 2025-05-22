#include <fstream>
#include <iostream>

#include <apex_mr/StabilityScore.h>
#include <apex_mr/TaskAssignment.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <planner.h>
#include <adg.h>
#include <logger.h>
#include <lego/Lego.hpp>
//#define OLD_IK_METHOD

const std::string PLANNING_GROUP = "dual_arms";

template<typename T>
using vec = std::vector<T>;

template<typename T>
using vec2d = std::vector<std::vector<T>>;

template<typename T>
using vec3d = std::vector<std::vector<std::vector<T>>>;

using vecgoal = std::vector<lego_manipulation::math::VectorJd>;

class TaskAssignment {
public:
    TaskAssignment(const std::string &output_dir,
                const std::string &task_name,
                const std::vector<std::string> &group_names,
                const std::vector<std::string> &eof_names,
                bool motion_plan_cost,
                bool check_stability,
                bool optimize_poses,
                bool print_debug) : 
        output_dir_(output_dir), group_names_(group_names), eof_names_(eof_names), task_name_(task_name), 
            motion_plan_cost_(motion_plan_cost), check_stability_(check_stability), optimize_poses_(optimize_poses),
            print_debug_(print_debug) {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model_ = robot_model_loader.getModel();
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        kinematic_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
        kinematic_state_->setToDefaultValues();

        ros::Duration(0.3).sleep();
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

        planning_scene_diff_client = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();

        instance_ = std::make_shared<MoveitInstance>(kinematic_state_, move_group_->getName(), planning_scene_);
        instance_->setNumberOfRobots(2);
        instance_->setRobotNames({"left_arm", "right_arm"});
        instance_->setRobotDOF(0, 7);
        instance_->setRobotDOF(1, 7);
        instance_->setPlanningSceneDiffClient(planning_scene_diff_client);

        stability_score_client_ = nh_.serviceClient<apex_mr::StabilityScore>("stability_score");
        assignment_client_ = nh_.serviceClient<apex_mr::TaskAssignment>("task_assignment");

        if (!boost::filesystem::exists(output_dir_)) {
            boost::filesystem::create_directories(output_dir_);
        }
    }
    

    bool setLegoFactory(const std::string &config_fname, const std::string &root_pwd, const std::string &task_fname)
    {
        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        std::string r1_DH_fname = root_pwd + config["r1_DH_fname"].asString();
        std::string r1_DH_tool_fname = root_pwd + config["r1_DH_tool_fname"].asString();
        std::string r1_DH_tool_assemble_fname = root_pwd + config["r1_DH_tool_assemble_fname"].asString();
        std::string r1_DH_tool_disassemble_fname = root_pwd + config["r1_DH_tool_disassemble_fname"].asString();
        std::string r1_DH_tool_alt_fname = root_pwd + config["r1_DH_tool_alt_fname"].asString();
        std::string r1_DH_tool_alt_assemble_fname = root_pwd + config["r1_DH_tool_alt_assemble_fname"].asString();
        std::string r1_DH_tool_handover_assemble_fname = root_pwd + config["r1_DH_tool_handover_assemble_fname"].asString();
        std::string r1_robot_base_fname = root_pwd + config["Robot1_Base_fname"].asString();
        std::string r2_DH_fname = root_pwd + config["r2_DH_fname"].asString();
        std::string r2_DH_tool_fname = root_pwd + config["r2_DH_tool_fname"].asString();
        std::string r2_DH_tool_assemble_fname = root_pwd + config["r2_DH_tool_assemble_fname"].asString();
        std::string r2_DH_tool_disassemble_fname = root_pwd + config["r2_DH_tool_disassemble_fname"].asString();
        std::string r2_DH_tool_alt_fname = root_pwd + config["r2_DH_tool_alt_fname"].asString();
        std::string r2_DH_tool_alt_assemble_fname = root_pwd + config["r2_DH_tool_alt_assemble_fname"].asString();
        std::string r2_DH_tool_handover_assemble_fname = root_pwd + config["r2_DH_tool_handover_assemble_fname"].asString();
        std::string r2_robot_base_fname = root_pwd + config["Robot2_Base_fname"].asString();

        std::string plate_calibration_fname = root_pwd + config["plate_calibration_fname"].asString();
        std::string env_setup_fname = root_pwd + config["env_setup_folder"].asString() + "env_setup_" + task_name_ + ".json";
        std::string lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();

        task_fname_ = task_fname;
        std::ifstream task_file(task_fname_, std::ifstream::binary);
        task_file >> task_json_;
        num_tasks_ = task_json_.size();
        remove_brick_seq();

        std::string world_base_fname = root_pwd + config["world_base_fname"].asString();

        bool assemble = config["Start_with_Assemble"].asBool();
        twist_rad_ = config["Twist_Deg"].asInt() * M_PI / 180.0;
        handover_twist_rad_ = config["Handover_Twist_Deg"].asInt() * M_PI / 180.0;

        set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        lego_ptr_ = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr_->setup(env_setup_fname, lego_lib_fname, plate_calibration_fname, assemble, task_json_, world_base_fname,
                        r1_DH_fname, r1_DH_tool_fname, r1_DH_tool_disassemble_fname, r1_DH_tool_assemble_fname, 
                        r1_DH_tool_alt_fname, r1_DH_tool_alt_assemble_fname, 
                        r1_DH_tool_handover_assemble_fname, r1_robot_base_fname, 
                        r2_DH_fname, r2_DH_tool_fname, r2_DH_tool_disassemble_fname, r2_DH_tool_assemble_fname,
                        r2_DH_tool_alt_fname, r2_DH_tool_alt_assemble_fname, 
                        r2_DH_tool_handover_assemble_fname, r2_robot_base_fname,
                        set_state_client_);
        return true;
    }

    void remove_brick_seq() {
        for (int i = 0; i < num_tasks_; i++) {
            int task_idx = i + 1;
            // calculate the ik for the target pose
            auto & cur_graph_node = task_json_[std::to_string(task_idx)];
            if (cur_graph_node.isMember("brick_seq")) {
                cur_graph_node.removeMember("brick_seq");
            }
        }
    }

    bool initLegoPositions() {
        if (lego_ptr_ == nullptr) {
            ROS_ERROR("Lego pointer is not initialized");
            return false;
        }

        std::vector<std::string> brick_names = lego_ptr_->get_brick_names();
        addMoveitCollisionObject("table");
        instance_->setObjectColor("table", 1.0, 1.0, 1.0, 1.0);
        for (const auto & name : brick_names) {
            addMoveitCollisionObject(name);
        }

        return true;

    }

    void addMoveitCollisionObject(const std::string &name) {
        Object obj = getLegoStart(name);

        // add the object to the planning scene
        instance_->addMoveableObject(obj);
        instance_->updateScene();

        instance_->setCollision(name, name, true);
        instance_->updateScene();
        
        log("Added collision object " + name + " at " + std::to_string(obj.x) + " " + std::to_string(obj.y) 
            + " " + std::to_string(obj.z) + " to world frame", LogLevel::DEBUG);
    }

    void removeMoveitCollisionObject(const std::string &name) {
        instance_->removeObject(name);
        instance_->updateScene();

        log("Removed collision object " + name, LogLevel::DEBUG);
    }

    void attachMoveitCollisionObject(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose) {
        instance_->attachObjectToRobot(name, robot_id, link_name, pose);
        instance_->updateScene();

        log("Attached collision object " + name + " to " + link_name, LogLevel::DEBUG);
    }

    void detachMoveitCollisionObject(const std::string &name, const RobotPose &pose) {
        instance_->detachObjectFromRobot(name, pose);
        instance_->updateScene();

        log("Detached collision object " + name, LogLevel::DEBUG);
    }

    Object getLegoStart(const std::string &brick_name) {
        Object obj;
        
        // define the object
        obj.name = brick_name;
        obj.state = Object::State::Static;
        obj.parent_link = "world";
        obj.shape = Object::Shape::Box;

        // get the starting pose and size
        geometry_msgs::Pose box_pose;
        if (brick_name == "table") {
            lego_ptr_->get_table_size(obj.length, obj.width, obj.height);
            box_pose = lego_ptr_->get_table_pose();

        } else {
            lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);
            box_pose = lego_ptr_->get_init_brick_pose(brick_name);
        }
        obj.x = box_pose.position.x;
        obj.y = box_pose.position.y;
        obj.z = box_pose.position.z - obj.height/2;
        obj.qx = box_pose.orientation.x;
        obj.qy = box_pose.orientation.y;
        obj.qz = box_pose.orientation.z;
        obj.qw = box_pose.orientation.w; 

        return obj;
    }

    Object getLegoTarget(int task_idx) {
        auto cur_graph_node =  task_json_[std::to_string(task_idx)];
        std::vector<std::string> brick_names = lego_ptr_->get_brick_names_by_type(cur_graph_node["brick_id"].asInt());
        if (brick_names.size() == 0) {
            ROS_ERROR("No bricks found for task %d", task_idx);
            return Object();
        }
        auto brick_name = brick_names[0];

        Eigen::Matrix4d brick_pose_mtx;
        lego_ptr_->calc_bric_asssemble_pose(brick_name, cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(),
                 cur_graph_node["z"].asInt(), cur_graph_node["ori"].asInt(), brick_pose_mtx);
        
        Object obj;
        obj.name = "b" + std::to_string(cur_graph_node["brick_id"].asInt()) + "_t" + std::to_string(task_idx);
        // define the object
        obj.state = Object::State::Static;
        obj.parent_link = "world";
        obj.shape = Object::Shape::Box;
        lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);

        obj.x = brick_pose_mtx(0, 3);
        obj.y = brick_pose_mtx(1, 3);
        obj.z = brick_pose_mtx(2, 3) - obj.height/2;
        Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
        obj.qx = quat.x();
        obj.qy = quat.y();
        obj.qz = quat.z();
        obj.qw = quat.w();

        return obj;
    }

    bool setCollision(const std::string& object_id, const std::string& link_name, bool allow) {
        instance_->setCollision(object_id, link_name, allow);
        instance_->updateScene();

        if (allow) {
            log("Allow collision between " + object_id + " and " + link_name, LogLevel::DEBUG);
        }
        else {
            log("Disallow collision between " + object_id + " and " + link_name, LogLevel::DEBUG);
        }

        return true;
    }
 
    void calculateIKforLego(const Eigen::MatrixXd& T, const Eigen::MatrixXd & home_q,
            int robot_id, int fk_type, bool check_collision, lego_manipulation::math::VectorJd& joint_q, bool &reachable) {
        if (!reachable) {
            return;
        }

        auto tic = std::chrono::high_resolution_clock::now();

#ifdef OLD_IK_METHOD
        if (robot_id == 0) {
            if (fk_type == 0) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
            }
            else if (fk_type == 1) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
            }
            else if (fk_type == 2) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
            }
            else if (fk_type == 3) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
            }
            else if (fk_type == 4) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
            }
            
        }
        else {
            if (fk_type == 0) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
            }
            else if (fk_type == 1) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
            }
            else if (fk_type == 2) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
            }
            else if (fk_type == 3) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
            }
            else if (fk_type == 4) {
                joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
            }
        }
        reachable &= (joint_q - home_q).norm() > 1e-6;
#else
        if (robot_id == 0) {
            if (fk_type == 0) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 
                                        lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_);
            }
            else if (fk_type == 1) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), 
                                        lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_assemble_inv_r1(), 0, IK_status_);
            }
            else if (fk_type == 2) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), 
                                        lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_disassemble_inv_r1(), 0, IK_status_);
            }
            else if (fk_type == 3) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), 
                                        lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_alt_inv_r1(), 0, IK_status_);
            }
            else if (fk_type == 4) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), 
                                        lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_alt_assemble_inv_r1(), 0, IK_status_);
            }
        }
        else {
            if (fk_type == 0) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 
                                        lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_inv_r2(), 0, IK_status_);
            }
            else if (fk_type == 1) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), 
                                        lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_assemble_inv_r2(), 0, IK_status_);
            }
            else if (fk_type == 2) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), 
                                        lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_disassemble_inv_r2(), 0, IK_status_);
            }
            else if (fk_type == 3) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), 
                                        lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_alt_inv_r2(), 0, IK_status_);
            }
            else if (fk_type == 4) {
                joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), 
                                        lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_alt_assemble_inv_r2(), 0, IK_status_);
            }
        }

        reachable = IK_status_;
#endif
        if (check_collision && reachable) {
            RobotPose pose = instance_->initRobotPose(robot_id);
            for (int i = 0; i < 6; i++) {
                pose.joint_values[i] = joint_q(i, 0) / 180.0 * M_PI;
            }
            bool hasCollision = instance_->checkCollision({pose}, false, print_debug_);
            reachable &= !hasCollision;
        }
        auto toc = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() / 1000.0;
        ik_reachability_time_ += duration;

    }

    void visualize_robot_pose(const lego_manipulation::math::VectorJd &joint_q, int robot_id) {
        RobotPose pose = instance_->initRobotPose(robot_id);
        for (int i = 0; i < 6; i++) {
            pose.joint_values[i] = joint_q(i, 0) / 180.0 * M_PI;
        }
        instance_->moveRobot(robot_id, pose);
        instance_->updateScene();
    }
    
    double calculateCost(const lego_manipulation::math::VectorJd &q1, const lego_manipulation::math::VectorJd &q2, int robot_id) {
        return calculateCostL1(q1, q2, robot_id);
    }


    bool validateInterpolation(const lego_manipulation::math::VectorJd &q1, const lego_manipulation::math::VectorJd &q2, int robot_id) {
        RobotPose pose1 = instance_->initRobotPose(robot_id);
        RobotPose pose2 = instance_->initRobotPose(robot_id);
        for (int i = 0; i < 6; i++) {
            pose1.joint_values[i] = q1(i, 0) / 180.0 * M_PI;
            pose2.joint_values[i] = q2(i, 0) / 180.0 * M_PI;
        }
        bool valid = instance_->connect(pose1, pose2, 0.1, print_debug_);

        return valid;
    }

    double calculateCostL1(const lego_manipulation::math::VectorJd &q1, const lego_manipulation::math::VectorJd &q2, int robot_id) {
        double cost = 0;
        for (int i = 0; i < 6; i++) {
            cost += std::abs(q1(i, 0) - q2(i, 0));
        }
        return cost;
    }

    std::vector<std::string> find_above_bricks(const std::string &brick_name, bool recursive) {
        std::vector<std::string> above_bricks = lego_ptr_->get_brick_above(brick_name);
        if (recursive) {
            size_t size = above_bricks.size();
            for (size_t i = 0; i < size; i++) {
                std::vector<std::string> tmp = find_above_bricks(above_bricks[i], true);
                above_bricks.insert(above_bricks.end(), tmp.begin(), tmp.end());
            }
        }
        return above_bricks;
    }

    bool isPressPtInBound(const std::string &brick_name, int press_side, int press_offset) {
        return lego_ptr_->is_press_pt_in_bound(brick_name, press_side, press_offset);
    }

    void calculatePickGoals(const std::string &brick_name, int press_side, int press_offset, bool &r1_reachable, bool &r2_reachable,
                            vecgoal &r1_goals, vecgoal &r2_goals) {
        lego_manipulation::math::VectorJd r1_pick_tilt_up, r1_pick_up, r1_pick, r1_pick_twist, r1_pick_twist_up;
        lego_manipulation::math::VectorJd r2_pick_tilt_up, r2_pick_up, r2_pick, r2_pick_twist, r2_pick_twist_up;

        // pick tilt up
        Eigen::MatrixXd cart_T = Eigen::MatrixXd::Identity(4, 4);
        lego_ptr_->calc_brick_grab_pose(brick_name, true, 1,
                            -1, -1, -1, -1, press_side, press_offset, cart_T);
        Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
        offset_T.col(3) << pick_offset(3), pick_offset(4), pick_offset(5) - abs(pick_offset(5)), 1;
        offset_T = cart_T * offset_T;
        calculateIKforLego(offset_T, home_q, 0, 0, false, r1_pick_tilt_up, r1_reachable);
        calculateIKforLego(offset_T, home_q, 1, 0, false, r2_pick_tilt_up, r2_reachable);

        // pick up
        Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
        up_T.col(3) << 0, 0, pick_offset(5), 1;
        up_T = cart_T * up_T;
        if (r1_reachable) {
            calculateIKforLego(up_T, r1_pick_tilt_up, 0, 0, false, r1_pick_up, r1_reachable);
        }
        if (r2_reachable) {
            calculateIKforLego(up_T, r2_pick_tilt_up, 1, 0, false, r2_pick_up, r2_reachable);
        }

        // pick pose
        if (r1_reachable) {
            calculateIKforLego(cart_T, r1_pick_up, 0, 0, false, r1_pick, r1_reachable);
        }
        if (r2_reachable) {
            calculateIKforLego(cart_T, r2_pick_up, 1, 0, false, r2_pick, r2_reachable);
        }
        // pick twist
        Eigen::Matrix4d twist_T = Eigen::MatrixXd::Identity(4, 4);
        twist_T.block(0, 0, 3, 3) << twist_R_pick;
        if (r1_reachable) {
            cart_T = lego_manipulation::math::FK(r1_pick, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), false);
            cart_T = cart_T * twist_T;
            calculateIKforLego(cart_T, r1_pick, 0, 2, false, r1_pick_twist, r1_reachable);
        }
        if (r2_reachable) {
            cart_T = lego_manipulation::math::FK(r2_pick, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), false);
            cart_T = cart_T * twist_T;
            calculateIKforLego(cart_T, r2_pick, 1, 2, false, r2_pick_twist, r2_reachable);
        }
        // pick twist up
        if (r1_reachable) {
            cart_T = lego_manipulation::math::FK(r1_pick_twist, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
            cart_T(2, 3) = cart_T(2, 3) + 0.015;
            calculateIKforLego(cart_T, r1_pick_twist, 0, 1, false, r1_pick_twist_up, r1_reachable);
        }
        if (r2_reachable) {
            cart_T = lego_manipulation::math::FK(r2_pick_twist, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
            cart_T(2, 3) = cart_T(2, 3) + 0.015;
            calculateIKforLego(cart_T, r2_pick_twist, 1, 1, false, r2_pick_twist_up, r2_reachable);
        }

        r1_goals = {r1_pick_tilt_up, r1_pick_up, r1_pick, r1_pick_twist, r1_pick_twist_up};
        r2_goals = {r2_pick_tilt_up, r2_pick_up, r2_pick, r2_pick_twist, r2_pick_twist_up};
        
    }

    bool calculateCostMatrix() {
        // initialize home pose
        home_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
        receive_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
        home_receive_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
        home_handover_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0;
        home_receive_q.col(0) << 0, 0, 0, 0, 0, 180;
        receive_q.col(0) << 0, 0, 0, 0, 0, 180;
        home_handover_q.col(0) << 0, 0, 0, 0, -90, 0;
        
        // home_q
        Eigen::Matrix4d home_T = lego_manipulation::math::FK(home_q, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), false);
        home_T.col(3) << 0.2, 0, 0.4, 1; // Home X, Y, Z in base frame of the Flange
        home_T = lego_ptr_->world_base_frame() * home_T;
        home_q = lego_manipulation::math::IK(home_q, home_T.block(0, 3, 3, 1), home_T.block(0, 0, 3, 3),
                                             lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5); // Home
        
        // Home to receive
        Eigen::Matrix4d home_receive_T = lego_manipulation::math::FK(home_receive_q, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), false);
        home_receive_T.col(3) << 0.375, 0, 0.35, 1;
        home_receive_T = lego_ptr_->world_base_frame() * home_receive_T; 
        home_receive_q = lego_ptr_->IK(home_receive_q, home_receive_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_);
        assert(IK_status_);

        // Receive 
        Eigen::Matrix4d receive_T = lego_manipulation::math::FK(receive_q, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), false);
        receive_T.col(3) << 0.45, 0, 0.35, 1;
        receive_T = lego_ptr_->world_base_frame() * receive_T; 
        receive_q = lego_ptr_->IK(receive_q, receive_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_);
        assert(IK_status_);

        // Home to handover
        Eigen::Matrix4d home_handover_T = lego_manipulation::math::FK(home_handover_q, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), false);
        home_handover_T.col(3) << 0.2, 0, 0.5, 1;
        home_handover_T = lego_ptr_->world_base_frame() * home_handover_T; 
        home_handover_q = lego_ptr_->IK(home_handover_q, home_handover_T, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_ee_inv_r1(), 0, IK_status_);


        // initialize support_t1, support_t2
        y_n90 << 0, 0, -1, 0, 
                0, 1, 0, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;
        y_s90 << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
        z_180 << -1, 0, 0, 0,
                 0, -1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
        Eigen::MatrixXd support_T2 = home_T * y_n90 * z_180;
        Eigen::MatrixXd support_pre_T2 = home_T * y_n90 * z_180;
        Eigen::MatrixXd support_T1 = home_T * y_s90;
        Eigen::MatrixXd support_pre_T1 = home_T * y_s90;

        // Attack angle
        pick_offset = Eigen::MatrixXd::Zero(6, 1);
        pick_offset << -0.005, 0.005, -0.005,  // place brick offset
                       -0.005, 0.005, -0.0028; // grab brick offset
        
        // Twist pose
        twist_R_pick = Eigen::MatrixXd::Identity(3, 3);
        twist_R_place = Eigen::MatrixXd::Identity(3, 3);
        twist_R_pick << cos(twist_rad_), 0, sin(twist_rad_), 
                    0, 1, 0, 
                    -sin(twist_rad_), 0, cos(twist_rad_);
        twist_R_place << cos(-twist_rad_), 0, sin(-twist_rad_), 
                    0, 1, 0, 
                    -sin(-twist_rad_), 0, cos(-twist_rad_);
        std::vector<std::string> brick_names = lego_ptr_->get_active_bricks_names();
        int num_bricks = brick_names.size();

        // initialize the cost matrix we want to calculate for milp solver'
        int num_sides = 4;
        int num_grasps = num_sides * num_offsets_;
        vec3d<double> cost_matrix_a(num_tasks_, vec2d<double>(num_bricks, vec<double>(num_grasps, 1000000)));
        vec3d<double> cost_matrix_b(num_tasks_, vec2d<double>(num_bricks, vec<double>(num_grasps, 1000000)));
        vec2d<double> support_matrix_a(num_tasks_, vec<double>(num_grasps, 1000000));
        vec2d<double> support_matrix_b(num_tasks_, vec<double>(num_grasps, 1000000));
        vec2d<int> delta_matrix(num_tasks_, vec<int>(num_bricks, 0));
        vec2d<int> precedence;
        vec2d<int> sup_required(num_tasks_, vec<int>(2, 0));

        // calculate the ik for each lego block's initial pose
        vec2d<bool> lego_r1_reachable(num_bricks, vec<bool>(num_grasps, false));
        vec2d<bool> lego_r2_reachable(num_bricks, vec<bool>(num_grasps, false));

        r1_lego_goals = vec2d<vecgoal>(num_bricks, vec<vecgoal>(num_grasps));
        r2_lego_goals = vec2d<vecgoal>(num_bricks, vec<vecgoal>(num_grasps));
        
        vec2d<double> cost_pick_r1(num_bricks, vec<double>(num_grasps, 1000000));
        vec2d<double> cost_pick_r2(num_bricks, vec<double>(num_grasps, 1000000));

        r1_task_goals = vec2d<vecgoal>(num_tasks_, vec<vecgoal>(num_grasps));
        r2_task_goals = vec2d<vecgoal>(num_tasks_, vec<vecgoal>(num_grasps));

        r1_support_goals = vec2d<vecgoal>(num_tasks_, vec<vecgoal>(num_grasps));
        r2_support_goals = vec2d<vecgoal>(num_tasks_, vec<vecgoal>(num_grasps));

        bool handover_feasible = calculateHandoverPoses(0, r1_handover_goals);
        assert(handover_feasible);      
        handover_feasible = calculateHandoverPoses(1, r2_handover_goals);
        assert(handover_feasible);

        double cost_receive = calculateCost(home_q, receive_q, 0);
        double cost_handover = calculateCost(home_q, r1_handover_goals[0], 0);

        for (int i = 0; i < num_bricks; i++) {
            std::string brick_name = brick_names[i];
            for (int g = 0; g < num_grasps; g++) {
                int press_side = g / num_offsets_ + 1;
                int press_offset = g % num_offsets_;
                if (!isPressPtInBound(brick_name, press_side, press_offset)) {
                    continue;
                }
                log("Calculating IK for brick: " + std::to_string(i) + " " + brick_name 
                    + " press side " + std::to_string(press_side) + " offset " + std::to_string(press_offset), LogLevel::INFO);

                bool r1_reachable = true, r2_reachable = true;
                calculatePickGoals(brick_name, press_side, press_offset, r1_reachable, r2_reachable,
                                    r1_lego_goals[i][g], r2_lego_goals[i][g]);
                lego_r1_reachable[i][g] = r1_reachable;
                lego_r2_reachable[i][g] = r2_reachable;
                log("Robot 1 reachability " + std::to_string(r1_reachable) + " Robot 2 reachability " + std::to_string(r2_reachable), LogLevel::DEBUG);
            }

            // find any blocks on top, add precedence
            std::vector<std::string> above_bricks = find_above_bricks(brick_name, false);
            for (auto & above_brick : above_bricks) {
                int above_idx = std::find(brick_names.begin(), brick_names.end(), above_brick) - brick_names.begin();
                precedence.push_back({above_idx, i});
            }
        }
    
        // calculate the cost of picking each lego block, assuming preceding lego blocks are already picked
        for (int i = 0; i < num_bricks; i++) {
            // find all preceding lego blocks
            std::vector<std::string> above_bricks = find_above_bricks(brick_names[i], true);
            for (auto & above_brick : above_bricks) {
                // remove the block from planning scene
                removeMoveitCollisionObject(above_brick);
            }

            for (int g = 0; g < num_grasps; g++) {
                log("Calculating Cost for brick: " + std::to_string(i) + " " + brick_names[i]
                    + " grasp " + std::to_string(g), LogLevel::INFO);
                if (lego_r1_reachable[i][g]) {
                    cost_pick_r1[i][g] = calculateCost(home_q, r1_lego_goals[i][g][2], 0);
                }
                if (lego_r2_reachable[i][g]) {
                    cost_pick_r2[i][g] = calculateCost(home_q, r2_lego_goals[i][g][2], 1);
                }
            }

            for (auto & above_brick : above_bricks) {
                // add the block back to planning scene
                addMoveitCollisionObject(above_brick);
            }
        }

        for (int i = 0; i < num_tasks_; i++) {
            int task_idx = i + 1;
            // calculate the ik for the target pose
            auto cur_graph_node = task_json_[std::to_string(task_idx)];
            int brick_id = cur_graph_node["brick_id"].asInt();
            int manip_type = 0;
            if (cur_graph_node["manipulate_type"].isInt()) {
                manip_type = cur_graph_node["manipulate_type"].asInt();
            } 

            // update world grid
            world_grid_ = lego_ptr_->gen_world_grid_from_graph(task_json_, i, 48, 48, 48);

            // move the lego brick to the target pose
            Object obj = getLegoTarget(task_idx);

            instance_->addMoveableObject(obj);
            instance_->updateScene();
            instance_->setCollision(obj.name, eof_names_[0], true);
            instance_->updateScene();
            instance_->setCollision(obj.name, eof_names_[1], true);
            instance_->updateScene();

            std::vector<int> r1_press_poses, r2_press_poses;
            std::vector<vecgoal> r1_goals, r2_goals;
            
            bool r1_stable = findBestPlacePoses(task_idx, 0, brick_names, cur_graph_node, manip_type, r1_press_poses, r1_goals);
            bool r2_stable = findBestPlacePoses(task_idx, 1, brick_names, cur_graph_node, manip_type, r2_press_poses, r2_goals);
            
            if (r1_press_poses.size() == 0 && r2_press_poses.size() == 0) {
                log("No press sides found for task with either robot " + std::to_string(task_idx), LogLevel::ERROR);
                ros::Duration(30).sleep();
                continue;
            }

            instance_->setCollision(obj.name, eof_names_[0], false);
            instance_->updateScene();
            instance_->setCollision(obj.name, eof_names_[1], false);
            instance_->updateScene();

            bool r1_feasible = false, r2_feasible = false; 
            for (int p = 0; p < r1_press_poses.size(); p++) {
                int press_side = r1_press_poses[p] / num_offsets_ + 1;
                int press_offset = r1_press_poses[p] % num_offsets_;
                int g = r1_press_poses[p];
    
                log("Calculating IK and cost for robot 1 task " + std::to_string(task_idx) 
                    + " press side " + std::to_string(press_side) + " offset " + std::to_string(press_offset), LogLevel::INFO);
                
                r1_task_goals[i][g] = r1_goals[p];
                //visualize_robot_pose(r1_goals[p][2], 0);
                double cost_place_r1 = calculateCost(home_q, r1_goals[p][2], 0);
                                
                std::string req_type = "b" + std::to_string(brick_id) + "_";
                for (int t = 0; t < num_bricks; t++) {
                    std::string brick_name = brick_names[t];
                    // check if brick name start with the req_type
                    if (brick_name.find(req_type) == 0) {
                        delta_matrix[i][t] = 1;
                    }
                    else {
                        continue;
                    }

                    if (manip_type == 0) {
                        if (lego_r1_reachable[t][g]) {
                            cost_matrix_a[i][t][g] = cost_pick_r1[t][g] + cost_place_r1;
                        }
                    }
                    else {
                        if (lego_r2_reachable[t][g]) {
                            cost_matrix_a[i][t][g] = cost_pick_r2[t][g] + cost_handover + cost_place_r1;
                        }
                    }
                }
            
                if (!r1_stable || (manip_type == 1)) {
                    sup_required[i][0] = 1;

                    bool r2_found = findStableSupportPose(press_side, press_offset, cur_graph_node, 1, task_idx, support_T2, support_pre_T2, r2_support_goals[i][g]);
                    if (r2_found) {
                        double cost_sup_r2 = calculateCost(home_q, r2_support_goals[i][g][0], 1);
                        if (manip_type == 0) {
                            support_matrix_b[i][g] = cost_sup_r2;
                        }
                        else {
                            support_matrix_b[i][g] = cost_receive + cost_sup_r2;
                        }
                        r1_feasible = true;
                        //visualize_robot_pose(r2_support_goals[i][g][0], 1);
                        //ros::Duration(1).sleep();
                    }
                    else {
                        log("No stable support pose found for task " + std::to_string(task_idx) 
                            + " r1 press side " + std::to_string(press_side) + " offset " + std::to_string(press_offset), LogLevel::WARN);
                    }
                }
                else {
                    r1_feasible = true;
                }
            }


            for (int p = 0; p < r2_press_poses.size(); p++) {
                int press_side = r2_press_poses[p] / num_offsets_ + 1;
                int press_offset = r2_press_poses[p] % num_offsets_;
                int g = r2_press_poses[p];

                log("Calculating IK and cost for robot 2 task " + std::to_string(task_idx) 
                    + " press side " + std::to_string(press_side) + " offset " + std::to_string(press_offset), LogLevel::INFO);

                r2_task_goals[i][g] = r2_goals[p];
                //visualize_robot_pose(r2_goals[p][2], 1);
                double cost_place_r2 = calculateCost(home_q, r2_goals[p][2], 1);
                                
                std::string req_type = "b" + std::to_string(brick_id) + "_";
                for (int t = 0; t < num_bricks; t++) {
                    std::string brick_name = brick_names[t];
                    // check if brick name start with the req_type
                    if (brick_name.find(req_type) == 0) {
                        delta_matrix[i][t] = 1;
                    }
                    else {
                        continue;
                    }

                    if (manip_type == 0) {
                        if (lego_r2_reachable[t][g]) {
                            cost_matrix_b[i][t][g] = cost_pick_r2[t][g] + cost_place_r2;
                        }
                    }
                    else {
                        if (lego_r1_reachable[t][g]) {
                            cost_matrix_b[i][t][g] = cost_pick_r1[t][g] + cost_handover + cost_place_r2;
                        }
                    }
                }
            
                if (!r2_stable || (manip_type == 1)) {
                    sup_required[i][1] = 1;

                    bool r1_found = findStableSupportPose(press_side, press_offset, cur_graph_node, 0, task_idx, support_T1, support_pre_T1, r1_support_goals[i][g]);
                    if (r1_found) {
                        double cost_sup_r1 = calculateCost(home_q, r1_support_goals[i][g][0], 0);
                        if (manip_type == 0) {
                            support_matrix_a[i][g] = cost_sup_r1;
                        }
                        else {
                            support_matrix_a[i][g] = cost_receive + cost_sup_r1;
                        }
                        r2_feasible = true;
                        // visualize_robot_pose(r1_support_goals[i][g][1], 0);
                        // ros::Duration(1).sleep();
                    }
                    else {
                        log("No stable support pose found for task " + std::to_string(task_idx) 
                            + " r2 press side " + std::to_string(press_side) + " offset " + std::to_string(press_offset), LogLevel::WARN);
                    }
                }
                else {
                    r2_feasible = true;
                }
            }

            // find the minimum of cost[i] for each brick and grasp
            double min_cost = 1000000;
            for (int j = 0; j < num_bricks; j++) {
                for (int g = 0; g < num_grasps; g++) {
                    if (cost_matrix_a[i][j][g] < min_cost) {
                        min_cost = cost_matrix_a[i][j][g];
                    }
                    if (cost_matrix_b[i][j][g] < min_cost) {
                        min_cost = cost_matrix_b[i][j][g];
                    }
                }
            }
            if (min_cost > 999999.9 || (!r1_feasible && !r2_feasible)) {
                log("No feasible solution found for task " + std::to_string(task_idx), LogLevel::ERROR);
                ros::Duration(30).sleep();
            }


        }

        // print the cost matrix
        std::ofstream cost_file(output_dir_ + "/cost_matrix_a.csv");
        for (int j = 0; j < num_bricks; j++) {
            for (int g = 0; g < num_grasps; g++) {
                cost_file << brick_names[j] << "@g_" << (g) << ",";
            }
        }
        cost_file << std::endl;
        for (int i = 0; i < num_tasks_; i++) {
            for (int j = 0; j < num_bricks; j++) {
                for (int g = 0; g < num_grasps; g++) {
                    cost_file << cost_matrix_a[i][j][g] << ",";
                }
            }
            cost_file << std::endl;
        }
        cost_file.close();

        cost_file.open(output_dir_ + "/cost_matrix_b.csv");
        for (int j = 0; j < num_bricks; j++) {
            for (int g = 0; g < num_grasps; g++) {
                cost_file << brick_names[j] << "@g_" << (g) << ",";
            }
        }
        cost_file << std::endl;
        for (int i = 0; i < num_tasks_; i++) {
            for (int j = 0; j < num_bricks; j++) {
                for (int g = 0; g < num_grasps; g++) {
                    cost_file << cost_matrix_b[i][j][g] << ",";
                }
            }
            cost_file << std::endl;
        }

        // print the support matrix
        std::ofstream support_file(output_dir_ + "/support_matrix_a.csv");
        for (int g = 0; g < num_grasps; g++) {
            support_file << "g_" << (g) << ",";
        }
        support_file << std::endl;
        for (int j = 0; j < num_tasks_; j++) {
            for (int g = 0; g < num_grasps; g++) {
                support_file << support_matrix_a[j][g] << ",";
            }
            support_file << std::endl;
        }
        support_file.close();

        support_file.open(output_dir_ + "/support_matrix_b.csv");
        for (int g = 0; g < num_grasps; g++) {
            support_file << "g_" << (g) << ",";
        }
        support_file << std::endl;
        for (int j = 0; j < num_tasks_; j++) {
            for (int g = 0; g < num_grasps; g++) {
                support_file << support_matrix_b[j][g] << ",";
            }
            support_file << std::endl;
        }

        // print the delta matrix
        std::ofstream delta_file(output_dir_ + "/delta_matrix.csv");
        for (int j = 0; j < num_bricks; j++) {
            delta_file << brick_names[j] << ",";
        }
        delta_file << std::endl;
        for (int i = 0; i < num_tasks_; i++) {
            for (int j = 0; j < num_bricks; j++) {
                delta_file << delta_matrix[i][j] << ",";
            }
            delta_file << std::endl;
        }
        delta_file.close();

        // print the support_req matrix
        std::ofstream support_req_file(output_dir_ + "/support_req.csv");
        support_req_file << "r1,r2," << std::endl;
        for (int i = 0; i < num_tasks_; i++) {
            support_req_file << sup_required[i][0] << "," << sup_required[i][1] << "," << std::endl;
        }
        support_req_file.close();

        // print the precedence matrix
        std::ofstream precedence_file(output_dir_ + "/precedence.csv");
        for (int i = 0; i < precedence.size(); i++) {
            precedence_file << precedence[i][0] << "," << precedence[i][1] << "," << std::endl;
        }
        precedence_file.close();

        return true;
    }

    bool calculateDropPoses(const std::string &brick_name, const Json::Value &cur_graph_node, int press_side,
                            int press_offset, int attack_dir, int task_idx, int robot_id, 
                            vecgoal &drop_goal) {
        // calculate drop pose
        lego_manipulation::math::VectorJd r_offset_goal, r_drop_up_goal, r_drop_goal, r_drop_twist_goal, r_drop_twist_up_goal;
        bool reachable = true;
        
        int press_x, press_y, press_ori;
        int brick_x = cur_graph_node["x"].asInt();
        int brick_y = cur_graph_node["y"].asInt();
        int brick_z = cur_graph_node["z"].asInt();
        int brick_ori = cur_graph_node["ori"].asInt();
        int brick_type = cur_graph_node["brick_id"].asInt();
        lego_ptr_->get_press_pt(brick_x, brick_y, brick_type, brick_ori, press_side, press_offset, press_x, press_y, press_ori);    

        Eigen::MatrixXd cart_T = Eigen::MatrixXd::Identity(4, 4);
        lego_ptr_->calc_brick_grab_pose(brick_name, 1, 0, brick_x, brick_y, brick_z, brick_ori, press_side, press_offset,
                            cart_T);

        // offset pose
        Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
        offset_T.col(3) << pick_offset(0), pick_offset(1) * attack_dir, pick_offset(2) - abs(pick_offset(2)), 1;
        offset_T = cart_T * offset_T;
        calculateIKforLego(offset_T, home_q, robot_id, 0, true, r_offset_goal, reachable);

        if (reachable) {
            // drop up 
            Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
            up_T.col(3) << 0, 0, pick_offset(2), 1;
            up_T = cart_T * up_T;
            calculateIKforLego(up_T, r_offset_goal, robot_id, 0, true, r_drop_up_goal, reachable);
        }

        // twist release path must be collision free
        std::vector<std::string> side_bricks;
        auto world_grid = lego_ptr_->gen_world_grid_from_graph(task_json_, task_idx - 1, 48, 48, 48);
        
        lego_ptr_->get_lego_next(press_x, press_y, brick_z, press_side, brick_ori, brick_type, brick_name, world_grid, side_bricks);
        for (auto & side_brick : side_bricks) {
            log("Setting collision between " + side_brick + " and " + eof_names_[robot_id], LogLevel::DEBUG);
            instance_->setCollision(side_brick, eof_names_[robot_id], true);
        }

        if (reachable) {
            // drop pose
            calculateIKforLego(cart_T, r_drop_up_goal, robot_id, 0, true, r_drop_goal, reachable);
        }


        // drop twist
        if (reachable) {
            Eigen::Matrix4d twist_T = Eigen::MatrixXd::Identity(4, 4);
            twist_T.block(0, 0, 3, 3) << twist_R_place;
            if (robot_id == 0) {
                cart_T = lego_manipulation::math::FK(r_drop_goal, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
            }
            else {
                cart_T = lego_manipulation::math::FK(r_drop_goal, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
            }
            cart_T = cart_T * twist_T;
            calculateIKforLego(cart_T, r_drop_goal, robot_id, 1, true, r_drop_twist_goal, reachable);
        }

        if (reachable) {
          
            reachable &= validateInterpolation(r_drop_goal, r_drop_twist_goal, robot_id);
            log("Twist release path is " + std::to_string(reachable), LogLevel::DEBUG);
        }

        // drop twist up
        if (reachable) {
            if (robot_id == 0) {
                cart_T = lego_manipulation::math::FK(r_drop_twist_goal, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
            }
            else {
                cart_T = lego_manipulation::math::FK(r_drop_twist_goal, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
            }
            cart_T(2, 3) = cart_T(2, 3) + 0.015;
            calculateIKforLego(cart_T, r_drop_twist_goal, robot_id, 1, true, r_drop_twist_up_goal, reachable);                
        }

        for (auto & side_brick : side_bricks) {
            instance_->setCollision(side_brick, eof_names_[robot_id], false);
        }

        drop_goal = {r_offset_goal, r_drop_up_goal, r_drop_goal, r_drop_twist_goal, r_drop_twist_up_goal};

        return reachable;
    }

    bool calculateHandoverPoses(int robot_id, vecgoal &handover_goal) {
        // calculate handover pose for the support arm robot id
        lego_manipulation::math::VectorJd r_transfer_up_goal, r_transfer_down_goal, r_transfer_twist_goal, r_transfer_twist_up_goal;
        Eigen::MatrixXd cart_T = Eigen::MatrixXd::Identity(4, 4);

        log("Calculating handover poses for robot " + std::to_string(robot_id+1), LogLevel::DEBUG);

        if (robot_id == 0) {
            // the other robot is receiving, the hadover robot needs to place at the receiving robot
            cart_T = lego_manipulation::math::FK(receive_q, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(receive_q, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), false);
        }
        cart_T = cart_T * y_s90 * z_180;
        
        Eigen::Matrix4d up_T = cart_T;
        up_T(2, 3) = up_T(2, 3) + 0.015;

        bool reachable = true;

        // transfer up
        calculateIKforLego(up_T, home_handover_q, robot_id, 0, true, r_transfer_up_goal, reachable);

        if (reachable) {
            // transfer down
            calculateIKforLego(cart_T, r_transfer_up_goal, robot_id, 0, true, r_transfer_down_goal, reachable);
        }
        if (reachable) {
            // transfer twist
            if (robot_id == 0) {
                cart_T = lego_manipulation::math::FK(r_transfer_down_goal, lego_ptr_->robot_DH_tool_handover_assemble_r1(), lego_ptr_->robot_base_r1(), false);
            }
            else {
                cart_T = lego_manipulation::math::FK(r_transfer_down_goal, lego_ptr_->robot_DH_tool_handover_assemble_r2(), lego_ptr_->robot_base_r2(), false);
            }
            Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
            Eigen::MatrixXd twist_R_handover = Eigen::MatrixXd::Identity(3, 3);
            twist_R_handover << cos(-handover_twist_rad_), 0, sin(-handover_twist_rad_), 
                    0, 1, 0, 
                    -sin(-handover_twist_rad_), 0, cos(-handover_twist_rad_);
            twist_T.block(0, 0, 3, 3) << twist_R_handover;
            cart_T = cart_T * twist_T;
            calculateIKforLego(cart_T, r_transfer_down_goal, robot_id, 1, true, r_transfer_twist_goal, reachable);
        }
        if (reachable) {
            // transfer twist up
            if (robot_id == 0) {
                cart_T = lego_manipulation::math::FK(r_transfer_twist_goal, lego_ptr_->robot_DH_tool_handover_assemble_r1(), lego_ptr_->robot_base_r1(), false);
            }
            else {
                cart_T = lego_manipulation::math::FK(r_transfer_twist_goal, lego_ptr_->robot_DH_tool_handover_assemble_r2(), lego_ptr_->robot_base_r2(), false);
            }
            Eigen::Matrix4d up_T = cart_T;
            up_T(2, 3) = up_T(2, 3) + 0.015;
            calculateIKforLego(up_T, r_transfer_twist_goal, robot_id, 1, true, r_transfer_twist_up_goal, reachable);
        }
        handover_goal = {r_transfer_up_goal, r_transfer_down_goal, r_transfer_twist_goal, r_transfer_twist_up_goal};

        log("Handover poses for robot " + std::to_string(robot_id+1) + " is " + std::to_string(reachable), LogLevel::DEBUG);
        return reachable;
    }

    bool calculatePlacePoses(const std::string &brick_name, const Json::Value &cur_graph_node, int press_x, int press_y,
                            int press_z, int press_ori, int press_side, int attack_dir, int task_idx, int robot_id, 
                            vecgoal &place_goal) {
        // calculate place from bottom pose
        lego_manipulation::math::VectorJd r_place_tilt_down_pre, r_place_tilt_down, r_place_down, r_place_up, r_twist, r_twist_down;
        bool reachable = true;
        Eigen::Matrix4d cart_T = Eigen::Matrix4d::Identity(4, 4);
        lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z+2, press_ori, press_side, cart_T);
        
        // place tilt down pre
        cart_T = cart_T * y_s90 * z_180;
        Eigen::Matrix4d pre_T = Eigen::MatrixXd::Identity(4, 4);
        pre_T.col(3) << -(pick_offset(5) - abs(pick_offset(5))), attack_dir * (-pick_offset(4)), pick_offset(3) - 0.02, 1;
        pre_T = cart_T * pre_T;
        calculateIKforLego(pre_T, home_receive_q, robot_id, 3, true, r_place_tilt_down_pre, reachable);
        
        if (reachable) {
            // place tilt down
            Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
            offset_T.col(3) << -(pick_offset(5) - abs(pick_offset(5))), attack_dir * (-pick_offset(4)), pick_offset(3), 1;
            offset_T = cart_T * offset_T;
            calculateIKforLego(offset_T, r_place_tilt_down_pre, robot_id, 3, false, r_place_tilt_down, reachable);
        }

        if (reachable) {
            // place down
            Eigen::Matrix4d down_T = Eigen::MatrixXd::Identity(4, 4);
            down_T.col(3) << -pick_offset(5), 0, 0, 1;
            down_T = cart_T * down_T;
            calculateIKforLego(down_T, r_place_tilt_down, robot_id, 3, false, r_place_down, reachable);
        }
        
        if (reachable) {
            // place up
            calculateIKforLego(cart_T, r_place_down, robot_id, 3, true, r_place_up, reachable);
        }

        if (reachable) {
            // place twist
            Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
            twist_T.block(0, 0, 3, 3) << twist_R_pick;

            if (robot_id == 0) {
                cart_T = lego_manipulation::math::FK(r_place_up, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), false);
            }
            else {
                cart_T = lego_manipulation::math::FK(r_place_up, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), false);
            }
            cart_T = cart_T * twist_T;
            calculateIKforLego(cart_T, r_place_up, robot_id, 4, true, r_twist, reachable);
        }

        if (reachable) {
            // place twist down
            if (robot_id == 0) {
                cart_T = lego_manipulation::math::FK(r_twist, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), false);
            }
            else {
                cart_T = lego_manipulation::math::FK(r_twist, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), false);
            }
            Eigen::Matrix4d down_T = Eigen::MatrixXd::Identity(4, 4);
            down_T << 1, 0, 0, 0.015,
                        0, 1, 0, 0,
                        0, 0, 1, -0.015,
                        0, 0, 0, 1;
            down_T = cart_T * down_T;
            calculateIKforLego(down_T, r_twist, robot_id, 4, true, r_twist_down, reachable);
        }
        place_goal = {r_place_tilt_down_pre, r_place_tilt_down, r_place_down, r_place_up, r_twist, r_twist_down};
        
        return reachable;
    }

    bool findBestPlacePoses(int task_idx, int robot_id, const vec<std::string> &brick_names, 
            const Json::Value &cur_graph_node, int manip_type, vec<int> &press_poses, vec<vecgoal> &goals) {

        int brick_id = cur_graph_node["brick_id"].asInt();
        int brick_width, brick_height;
        lego_ptr_->get_brick_sizes_by_type(brick_id, brick_width, brick_height);
        int attack_dir = 1;
        if (cur_graph_node.isMember("attack_dir")) {
            attack_dir = cur_graph_node["attack_dir"].asInt();
        }

        std::string brick_name;
        for (auto & name : brick_names) {
            if (name.find("b" + std::to_string(brick_id) + "_") == 0) {
                brick_name = name;
                break;
            }
        }

        press_poses.clear();
        goals.clear();

        if (!optimize_poses_) {
            // check if the cur graph node has press_pt already
            if (cur_graph_node.isMember("press_side") && cur_graph_node.isMember("press_x")) {
                int press_side = cur_graph_node["press_side"].asInt();
                int press_offset = cur_graph_node["press_offset"].asInt();

                vecgoal goal;
                bool reachable;
                if (manip_type == 0) {
                    reachable = calculateDropPoses(brick_name, cur_graph_node, press_side, press_offset, attack_dir, task_idx, robot_id, goal);
                }
                else {
                    int press_x, press_y, press_z, press_ori;
                    press_x = cur_graph_node["press_x"].asInt();
                    press_y = cur_graph_node["press_y"].asInt();
                    press_z = cur_graph_node["press_z"].asInt();
                    press_ori = cur_graph_node["press_ori"].asInt();
                    reachable = calculatePlacePoses(brick_name, cur_graph_node, press_x, press_y, press_z, press_ori, press_side, attack_dir, task_idx, robot_id, goal);
                }
                if (reachable) {
                    goals.push_back(goal);
                    press_poses.push_back((press_side - 1) * num_offsets_ + press_offset);
                    bool stable = false;
                    if (cur_graph_node.isMember("support_x") && cur_graph_node["support_x"].asInt() == -1) {
                        stable = true;
                    }
                    return stable;
                }
                else {
                    log("Task " + std::to_string(task_idx) + " robot " + std::to_string(robot_id + 1) + " given press side " 
                        + std::to_string(press_side) + " offset " + std::to_string(press_offset) + " is not reachable.", LogLevel::WARN);
                    return false;
                }
            }
            else {
                log("Task " + std::to_string(task_idx) + " robot " + std::to_string(robot_id + 1) + " does not have press side and offset", LogLevel::ERROR);
                return false;
            }
        }
        else {
            // rank the press sides based on the stability
            vec<int> fes_press_poses;
            vec<int> stability;
            vec<vecgoal> fes_goals;
            for (int press_side = 1; press_side <= 4; press_side++)
            {
                for (int press_offset = 0; press_offset < num_offsets_; press_offset ++) {
                    // check if press side touches other existing bricks
                    if (!isPressPtInBound(brick_name, press_side, press_offset)) {
                        continue;
                    }
                    int press_pt_x, press_pt_y, press_ori, height, width;
                    lego_ptr_->get_press_pt(cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(), brick_id, 
                            cur_graph_node["ori"].asInt(), press_side, press_offset, press_pt_x, press_pt_y, press_ori);
                    int press_pt_z = cur_graph_node["z"].asInt();
                    log("Press pt " + std::to_string(press_pt_x) + " " + std::to_string(press_pt_y) + " " + std::to_string(press_pt_z) + " " + std::to_string(press_ori), LogLevel::DEBUG);
                    
                    if (press_pt_x < world_grid_.size() && press_pt_y < world_grid_[0].size() && !world_grid_[press_pt_x][press_pt_y][press_pt_z-1].empty()) {
                        continue;
                    }
                    int press_pt_x2 = (press_ori == 0) ? press_pt_x : press_pt_x + 1;
                    int press_pt_y2 = (press_ori == 0) ? press_pt_y + 1 : press_pt_y;
                    if (press_pt_x2 < world_grid_.size() && press_pt_y2 < world_grid_[0].size() && !world_grid_[press_pt_x2][press_pt_y2][press_pt_z-1].empty()) {                
                        continue;
                    }
                    
                    vecgoal goal;
                    bool reachable;
                    if (manip_type == 0) {
                        reachable = calculateDropPoses(brick_name, cur_graph_node, press_side, press_offset, attack_dir, task_idx, robot_id, goal);
                    }
                    else {
                        reachable = calculatePlacePoses(brick_name, cur_graph_node, press_pt_x, press_pt_y, press_pt_z-1, press_ori, press_side, attack_dir, task_idx, robot_id, goal);
                    }
                        // compute stability
                    if (reachable) {
                        if (manip_type == 0) {
                            bool sup_needed = checkSupportNeeded(cur_graph_node, press_side, press_offset, task_idx);
                            int press_side_score = (sup_needed) ? 0 : 100;
                            // press_side_score += checkPressSideStability(cur_graph_node, press_side, task_idx);
                            stability.push_back(press_side_score);
                        }
                        else {
                            stability.push_back(0);
                        }

                        fes_goals.push_back(goal);
                        fes_press_poses.push_back((press_side-1) * num_offsets_ + press_offset);
                    }
                }
            }
            
            // get the max stability
            if (fes_press_poses.empty()) {
                log("No feasible press sides found for task " + std::to_string(task_idx) + " robot " + std::to_string(robot_id + 1), LogLevel::WARN);
                return false;
            }

            int max_stability = *std::max_element(stability.begin(), stability.end());
            for (int i = 0; i < stability.size(); i++) {
                if (stability[i] == max_stability) {
                    press_poses.push_back(fes_press_poses[i]);
                    goals.push_back(fes_goals[i]);
                }
            }

            bool stable = false;
            if (max_stability >= 100) {
                stable = true;
            }

            log("robot " + std::to_string(robot_id + 1) + " task " + std::to_string(task_idx) + " have " + std::to_string(press_poses.size())
                +  (stable ? " stable" : " unstable" ) + " press sides ", LogLevel::INFO);
            return stable;
        }
    }

    int checkPressSideStability(const Json::Value &cur_graph_node, int press_side, int task_idx) {
        int brick_id = cur_graph_node["brick_id"].asInt();
        int ori = cur_graph_node["ori"].asInt();
        int brick_x = cur_graph_node["x"].asInt();
        int brick_y = cur_graph_node["y"].asInt();
        int brick_z = cur_graph_node["z"].asInt();
        int h, w;
        lego_ptr_->get_brick_sizes_by_type(brick_id, h, w);
        
        if (brick_z <= 1) {
            return 2;
        }

        int stability = 0;
        if (ori == 0) {
            if (press_side == 1) {
                for (int j = brick_y+w/2-1; j < brick_y+w/2+1; j++) {
                    if (!world_grid_[brick_x][j][brick_z-2].empty()) {
                        stability++;
                    }
                }
            }
            else if (press_side == 2) {
                for (int i = brick_x+h/2-1; i < brick_x+h/2+1; i++) {
                    if (!world_grid_[i][brick_y+w-1][brick_z-2].empty()) {
                        stability++; 
                    }
                }
            }
            else if (press_side == 3) {
                for (int i = brick_x+h/2-1; i < brick_x+h/2+1; i++) {
                    if (!world_grid_[i][brick_y][brick_z-2].empty()) {
                        stability++;
                    }
                }
            }
            else if (press_side == 4) {
                for (int j = brick_y+w/2-1; j < brick_y+w/2+1; j++) {
                    if (!world_grid_[brick_x+h-1][j][ brick_z-2].empty()) {
                        stability++;
                    }
                }
            }
        }
        else if (ori == 1) {
            std::swap(h, w);
            if (press_side == 1) {
                for (int i = brick_x+h/2-1; i < brick_x+h/2+1; i++) {
                    if (!world_grid_[i][brick_y][brick_z-2].empty()) {
                        stability++;
                    }
                }
            }
            else if (press_side == 2) {
                for (int j = brick_y+w/2-1; j < brick_y+w/2+1; j++) {
                    if (!world_grid_[brick_x][j][brick_z-2].empty()) {
                        stability++;
                    }
                }
            }
            else if (press_side == 3) {
                for (int j = brick_y+w/2-1; j < brick_y+w/2+1; j++) {
                    if (!world_grid_[brick_x+h-1][j][brick_z-2].empty()) {
                        stability++;
                    }
                }
            }
            else if (press_side == 4) {
                for (int i = brick_x+h/2-1; i < brick_x+h/2+1; i++) {
                    if (!world_grid_[i][brick_y+w-1][brick_z-2].empty()) {
                        stability++;
                    }
                }
            }
        }
        return stability;
            
    }

    bool checkSupportNeeded(const Json::Value &cur_graph_node, int press_side, int press_offset, int task_idx) 
    { 
        int press_pt_x, press_pt_y, press_ori;
        int press_pt_z = cur_graph_node["z"].asInt() + 1;
        lego_ptr_->get_press_pt(cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(), cur_graph_node["brick_id"].asInt(), 
                    cur_graph_node["ori"].asInt(), press_side, press_offset, press_pt_x, press_pt_y, press_ori);

        if (check_stability_) {
            // use gurobi to check physical stability
            bool stable = checkStability(press_pt_x, press_pt_y, press_pt_z, false, -1, -1, -1, press_ori, task_idx);
            return !stable;
        }
        else {
            return false;
        }
    }

    bool findStableSupportPose(int press_side, int press_offset, const Json::Value &cur_graph_node, int robot_id, int task_idx, 
            Eigen::MatrixXd &support_T, Eigen::MatrixXd &support_pre_T, vecgoal &r_sup_goal) {
        
        r_sup_goal = {Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1)};

        int brick_x = cur_graph_node["x"].asInt();
        int brick_y = cur_graph_node["y"].asInt();
        int ori = cur_graph_node["ori"].asInt();
        int brick_id = cur_graph_node["brick_id"].asInt();

        int press_pt_x, press_pt_y, press_ori;
        lego_ptr_->get_press_pt(brick_x, brick_y, brick_id, ori, press_side, press_offset, press_pt_x, press_pt_y, press_ori);

        Eigen::MatrixXd init_q = home_q;
#ifdef OLD_IK_METHOD
        init_q(4) = 30;
#endif

        // check if the cur graph node has press_pt already
        if (!optimize_poses_) {
            if (cur_graph_node.isMember("support_x")) {
                int support_x = cur_graph_node["support_x"].asInt();
                int support_y = cur_graph_node["support_y"].asInt();
                int support_z = cur_graph_node["support_z"].asInt();
                int support_ori = cur_graph_node["support_ori"].asInt();
                int manip_type = 0;
                if (cur_graph_node.isMember("manipulate_type")) {
                    manip_type = cur_graph_node["manipulate_type"].asInt();
                }
                
                bool reachable = true;

                if (manip_type == 0) {
                    Eigen::Matrix4d sup_T = Eigen::MatrixXd::Identity(4, 4);
                    lego_ptr_->support_pose(support_x, support_y, support_z, support_ori, sup_T);
 
                    // disable collision with nearby bricks
                    std::vector<std::string> side_bricks, side_low_bricks, above_bricks;
                    int sup_press_side, sup_brick_ori;
                    lego_ptr_->get_sup_side_ori(support_ori, sup_press_side, sup_brick_ori);
                    lego_ptr_->get_lego_next(support_x, support_y, support_z, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid_, side_bricks);
                    lego_ptr_->get_lego_next(support_x, support_y, support_z-1, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid_, side_low_bricks);
                    side_bricks.insert(side_bricks.end(), side_low_bricks.begin(), side_low_bricks.end());
                    lego_ptr_->get_lego_above(support_x, support_y, support_z, sup_brick_ori, 9, world_grid_, above_bricks);
                    
                    for (auto & above_brick : above_bricks) {
                        setCollision(above_brick, eof_names_[robot_id], true);
                    }
                    for (auto & side_brick : side_bricks) {
                        setCollision(side_brick, eof_names_[robot_id], true);
                    }

                    calculateIKforLego(sup_T, init_q, robot_id, 0, true, r_sup_goal[1], reachable);

                    // reenable collision
                    for (auto & above_brick : above_bricks) {
                        setCollision(above_brick, eof_names_[robot_id], false);
                    }
                    for (auto & side_brick : side_bricks) {
                        setCollision(side_brick, eof_names_[robot_id], false);
                    }

                    if (reachable) { 
                        // find nearby free pose
                        vec<double> dxs = {0, 0.001, -0.001};
                        vec<double> dys = {0, 0.001, -0.001};
                        for (double dx : dxs) {
                            for (double dy : dys) {
                                Eigen::Matrix4d sup_T_down = Eigen::MatrixXd::Identity(4, 4);
                                sup_T_down = sup_T;
                                sup_T_down(0, 3) += dx;
                                sup_T_down(1, 3) += dy;
                                sup_T_down(2, 3) += sup_down_offset;
                                reachable = true;
                                calculateIKforLego(sup_T_down, r_sup_goal[1], robot_id, 0, true, r_sup_goal[0], reachable);
                                if (reachable) {
                                    log("Found stable support pose for task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::DEBUG);
                                    return true;
                                }
                            }
                        }
                        log("No nearby pre-support pose found for  task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::WARN);
                        return false;
                    }
                }
                else {
                    // press from the top as support for placing from the bottom
                    Eigen::Matrix4d press_T = Eigen::MatrixXd::Identity(4, 4);
                    Eigen::Matrix4d press_up_T = press_T;
                    
                    // press up
                    if(support_ori == 0){
                        lego_ptr_->assemble_pose_from_top(support_x+1, support_y, support_z+1, 0, support_ori+1, press_T);
                        press_T(0, 3) = press_T(0, 3) + 0.002;
                    }
                    else if(support_ori == 1){
                        lego_ptr_->assemble_pose_from_top(support_x, support_y-1, support_z+1, 1, support_ori+1, press_T);
                        press_T(1, 3) = press_T(1, 3) - 0.002;
                    }
                    else if (support_ori == 2) {
                        lego_ptr_->assemble_pose_from_top(support_x, support_y+1, support_z+1, 1, support_ori+1, press_T);
                        press_T(1, 3) = press_T(1, 3) + 0.002;
                    }
                    else if (support_ori == 3) {
                        lego_ptr_->assemble_pose_from_top(support_x-1, support_y, support_z+1, 0, support_ori+1, press_T);
                        press_T(0, 3) = press_T(0, 3) - 0.002;
                    }
                    else {
                        log("Invalid support orientation " + std::to_string(support_ori), LogLevel::ERROR);
                        return false;
                    }
                    press_T(2, 3) = press_T(2, 3) - lego_ptr_->brick_height() + lego_ptr_->lever_wall_height() + lego_ptr_->knob_height();
                    press_up_T = press_T;
                    press_up_T(2, 3) = press_up_T(2, 3) + 0.005;

                    std::vector<std::string> side_bricks, below_bricks;
                    int sup_press_side, sup_brick_ori;
                    lego_ptr_->get_sup_side_ori(support_ori, sup_press_side, sup_brick_ori);
                    lego_ptr_->get_lego_next(support_x, support_y, support_z, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid_, side_bricks);
                    lego_ptr_->get_lego_below(support_x, support_y, support_z, sup_brick_ori, 9, world_grid_, below_bricks);
                    for (auto & side_brick : side_bricks) {
                        setCollision(side_brick, eof_names_[robot_id], true);
                    }
                    for (auto & below_brick : below_bricks) {
                        setCollision(below_brick, eof_names_[robot_id], true);
                    }
                    calculateIKforLego(press_T, home_q, robot_id, 0, true, r_sup_goal[1], reachable);
                    for (auto & side_brick : side_bricks) {
                        setCollision(side_brick, eof_names_[robot_id], false);
                    }
                    for (auto & below_brick : below_bricks) {
                        setCollision(below_brick, eof_names_[robot_id], false);
                    }
                    
                    if (reachable) {
                        calculateIKforLego(press_up_T, r_sup_goal[1], robot_id, 0, true, r_sup_goal[0], reachable);

                        if (reachable) {
                            log("Found stable press support pose for task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::DEBUG);
                            return true;
                        }
                        
                    }
                }
            }
            else {
                // raise error
                log("Task " + std::to_string(task_idx) + " robot " + std::to_string(robot_id + 1) + " does not have support pose specified", LogLevel::ERROR);
                return false;
            }
        }
        else { // optimize_poses_ is true
            int manip_type = 0;
            if (cur_graph_node.isMember("manipulate_type")) {
                manip_type = cur_graph_node["manipulate_type"].asInt();
            }

            if (manip_type == 0) { // Support from below (EE acts as a temporary brick)
                int press_pt_z = cur_graph_node["z"].asInt() + 1;
                vec<int> dxs = {0, 1, -1}; // X offset for the support tool
                vec<int> dys = {0, 1, -1}; // Y offset for the support tool
                vec<int> support_oris = {0, 1, 2, 3}; // Orientation of the support tool

                for (int dx : dxs) {
                    for (int dy : dys) {
                        for (int support_ori_tool : support_oris) {
                            // dz is downwards offset to find the Z coordinate for the EE's tip
                            
                            for (int dz_ee = 2; dz_ee <= 4; dz_ee++) { // dz_ee defines how far below press_pt_z the EE tip is
                                int support_x = press_pt_x + dx;
                                int support_y = press_pt_y + dy;
                                int support_z = press_pt_z - 1 - dz_ee; // Z-coordinate of the EE tip

                                if (support_z < 0 || support_z >= world_grid_[0][0].size() ||
                                    support_x < 0 || support_x >= world_grid_.size() ||
                                    support_y < 0 || support_y >= world_grid_[0].size()) {
                                    continue;
                                }

                                // Condition for valid EE support placement:
                                if (support_z == 0 || !world_grid_[support_x][support_y][support_z-1].empty()) { // E E tip cell must be empty
                                    continue;
                                }
                                if (world_grid_[support_x][support_y][support_z].empty()) { // Cell above EE tip must be occupied 
                                    continue;
                                }

                                std::vector<std::string> side_bricks, above_bricks;
                                int sup_press_side_for_ee, sup_brick_ori_for_ee_shape;
                                lego_ptr_->get_sup_side_ori(support_ori_tool, sup_press_side_for_ee, sup_brick_ori_for_ee_shape);
                                // Get bricks next to and above the EE's intended position
                                lego_ptr_->get_lego_next(support_x, support_y, support_z, sup_press_side_for_ee, sup_brick_ori_for_ee_shape, 9, "support_brick", world_grid_, side_bricks);
                                lego_ptr_->get_lego_above(support_x, support_y, support_z, sup_brick_ori_for_ee_shape, 9, world_grid_, above_bricks);

                                for (auto & above_brick : above_bricks) setCollision(above_brick, eof_names_[robot_id], true);
                                for (auto & side_brick : side_bricks) setCollision(side_brick, eof_names_[robot_id], true);

                                Eigen::Matrix4d sup_T = Eigen::MatrixXd::Identity(4, 4);
                                lego_ptr_->support_pose(support_x, support_y, support_z, support_ori_tool, sup_T);
                                log("Checking reachability for task: " + std::to_string(task_idx) + " at sup_x "
                                    + std::to_string(support_x) + " sup_y " + std::to_string(support_y) + " sup_z_ee_tip " + std::to_string(support_z)
                                    + " sup_ori_tool " + std::to_string(support_ori_tool), LogLevel::DEBUG);

                                bool reachable = true;
                                calculateIKforLego(sup_T, init_q, robot_id, 0, true, r_sup_goal[1], reachable);

                                for (auto & above_brick : above_bricks) setCollision(above_brick, eof_names_[robot_id], false);
                                for (auto & side_brick : side_bricks) setCollision(side_brick, eof_names_[robot_id], false);

                                if (reachable) {
                                    log("Checking stability for task: " + std::to_string(task_idx) + " at sup_x "
                                        + std::to_string(support_x) + " sup_y " + std::to_string(support_y) + " sup_z_contact " + std::to_string(support_z - 1)
                                        + " sup_ori_tool " + std::to_string(support_ori_tool), LogLevel::DEBUG);
                                    // Stability check uses the Z of the contact point on the structure
                                    bool stable = checkStability(press_pt_x, press_pt_y, press_pt_z, true, support_x, support_y, support_z - 1, press_ori, task_idx);
                                    if (stable) {
                                        log("Found stable support pose for task: " + std::to_string(task_idx) + " at sup_x "
                                            + std::to_string(support_x) + " sup_y " + std::to_string(support_y) + " sup_z_ee_tip " + std::to_string(support_z)
                                            + " sup_ori_tool " + std::to_string(support_ori_tool), LogLevel::INFO);
                                        
                                        Eigen::Matrix4d sup_T_down = sup_T;
                                        sup_T_down(2, 3) += sup_down_offset; // sup_down_offset is typically negative (e.g., -0.001)
                                                                            
                                        calculateIKforLego(sup_T_down, r_sup_goal[1], robot_id, 0, true, r_sup_goal[0], reachable);

                                        if (reachable) {
                                            log("Found stable support pre pose for task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::DEBUG);
                                            return true;
                                        } else {
                                            log("No nearby pre-support pose found for task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1)
                                                + " for sup_x " + std::to_string(support_x) + " sup_y " + std::to_string(support_y) + " sup_z_ee_tip " + std::to_string(support_z)
                                                + " sup_ori_tool " + std::to_string(support_ori_tool), LogLevel::WARN);
                                        }
                                    }
                                }
                            } // end dz_ee
                        } // end support_ori_tool
                    } // end dy
                } // end dx
            } else { // manip_type == 1 (Press from top as support for placing from bottom)
                int press_pt_z = cur_graph_node["z"].asInt() - 1;
                vec<int> dx_brick_offsets = {0, 1, -1}; // Offset from cur_graph_node["x"] to find brick_to_press
                vec<int> dy_brick_offsets = {0, 1, -1}; // Offset from cur_graph_node["y"] to find brick_to_press
                vec<int> press_action_oris = {0, 3}; // Orientation of the pressing action (0-3)

                for (int dx_b : dx_brick_offsets) {
                    for (int dy_b : dy_brick_offsets) {
                        // Iterate Z layer of the brick to be pressed by the support arm.
                        for (int dz_b_offset = 3; dz_b_offset <= 4; ++dz_b_offset) {
                            int brick_to_press_x = press_pt_x + dx_b;
                            int brick_to_press_y = press_pt_y + dy_b;
                            int brick_to_press_z = press_pt_z + dz_b_offset;

                            if (brick_to_press_z <= 0 || brick_to_press_z >= world_grid_[0][0].size() ||
                                brick_to_press_x < 0 || brick_to_press_x >= world_grid_.size() ||
                                brick_to_press_y < 0 || brick_to_press_y >= world_grid_[0].size()) {
                                continue;
                            }
                            if (!world_grid_[brick_to_press_x][brick_to_press_y][brick_to_press_z-1].empty() // EE tip must be empty
                                || world_grid_[brick_to_press_x][brick_to_press_y][brick_to_press_z-2].empty()) { // Must be a brick to press one level below
                                continue;
                            }

                            for (int press_action_ori : press_action_oris) {
                                log("Testing support brick pose ("
                                    + std::to_string(brick_to_press_x) + "," + std::to_string(brick_to_press_y) + "," + std::to_string(brick_to_press_z) + ")"
                                    + " with press ori " + std::to_string(press_action_ori), LogLevel::DEBUG);
                                Eigen::Matrix4d press_T = Eigen::MatrixXd::Identity(4, 4);
                                Eigen::Matrix4d press_up_T = Eigen::MatrixXd::Identity(4, 4);

                                int eff_press_target_x = brick_to_press_x; // X-coord of brick for assemble_pose_from_top
                                int eff_press_target_y = brick_to_press_y; // Y-coord of brick for assemble_pose_from_top
                                int eff_press_target_z_arg = brick_to_press_z + 1; // Z-arg for assemble_pose_from_top (top surface)
                                int eff_press_target_ori_arg = 0; // Orientation of the brick being pressed
                                int eff_press_tool_side = press_action_ori + 1; // Tool's pressing side (1-4)

                                // Adjust effective target for assemble_pose_from_top based on press_action_ori
                                // This mimics the logic from the non-optimized branch for manip_type=1 support
                                if(press_action_ori == 0){ // Pressing brick at (brick_to_press_x+1, brick_to_press_y)
                                    eff_press_target_x = brick_to_press_x + 1; eff_press_target_ori_arg = 0;
                                } else if(press_action_ori == 1){ // Pressing brick at (brick_to_press_x, brick_to_press_y-1)
                                    eff_press_target_y = brick_to_press_y - 1; eff_press_target_ori_arg = 1;
                                } else if (press_action_ori == 2) { // Pressing brick at (brick_to_press_x, brick_to_press_y+1)
                                    eff_press_target_y = brick_to_press_y + 1; eff_press_target_ori_arg = 1;
                                } else if (press_action_ori == 3) { // Pressing brick at (brick_to_press_x-1, brick_to_press_y)
                                    eff_press_target_x = brick_to_press_x - 1; eff_press_target_ori_arg = 0;
                                }

                                lego_ptr_->assemble_pose_from_top(eff_press_target_x, eff_press_target_y, eff_press_target_z_arg,
                                                                eff_press_target_ori_arg, eff_press_tool_side, press_T);
                                
                                if(press_action_ori == 0) press_T(0, 3) += 0.002;
                                else if(press_action_ori == 1) press_T(1, 3) -= 0.002;
                                else if (press_action_ori == 2) press_T(1, 3) += 0.002;
                                else if (press_action_ori == 3) press_T(0, 3) -= 0.002;
                                
                                press_T(2, 3) = press_T(2, 3) - lego_ptr_->brick_height() + lego_ptr_->lever_wall_height() + lego_ptr_->knob_height();
                                press_up_T = press_T;
                                press_up_T(2, 3) += 0.005;

                                std::vector<std::string> side_bricks_collision, below_bricks_collision;
                                int col_check_press_side, col_check_brick_ori;
                                // Collision checks around the brick at (eff_press_target_x, eff_press_target_y, brick_to_press_z)
                                lego_ptr_->get_sup_side_ori(press_action_ori, col_check_press_side, col_check_brick_ori);
                                lego_ptr_->get_lego_next(eff_press_target_x, eff_press_target_y, brick_to_press_z, col_check_press_side, col_check_brick_ori, 9, "support_brick", world_grid_, side_bricks_collision);
                                lego_ptr_->get_lego_below(eff_press_target_x, eff_press_target_y, brick_to_press_z, col_check_brick_ori, 9, world_grid_, below_bricks_collision);
                                
                                for (auto & sb : side_bricks_collision) setCollision(sb, eof_names_[robot_id], true);
                                for (auto & bb : below_bricks_collision) setCollision(bb, eof_names_[robot_id], true);

                                bool reachable = true;
                                calculateIKforLego(press_T, init_q, robot_id, 0, true, r_sup_goal[1], reachable);

                                for (auto & sb : side_bricks_collision) setCollision(sb, eof_names_[robot_id], false);
                                for (auto & bb : below_bricks_collision) setCollision(bb, eof_names_[robot_id], false);

                                if (reachable) {
                                    calculateIKforLego(press_up_T, r_sup_goal[1], robot_id, 0, true, r_sup_goal[0], reachable);
                                    

                                    if (reachable) {
                                        log("Found press support pose for task: " + std::to_string(task_idx) + " pressing brick at ("
                                            + std::to_string(brick_to_press_x) + "," + std::to_string(brick_to_press_y) + "," + std::to_string(brick_to_press_z) + ")"
                                            + " with press_action_ori " + std::to_string(press_action_ori), LogLevel::INFO);
                                        return true;
                                    }
                                }
                            } // end press_action_ori
                        } // end dz_b_offset
                    } // end dy_b
                } // end dx_b
            } // end manip_type check
        } // end optimize_poses_
        return false;
    }

    bool findNearbyPreSupPose(int robot_id, int task_idx, const Eigen::MatrixXd &support_T, Eigen::MatrixXd &support_pre_T, vecgoal &r_sup_goal) {

        bool reachable;
        // find nearby free pose
        support_pre_T = support_T;
        support_pre_T(2, 3) += sup_down_offset;
        calculateIKforLego(support_pre_T, r_sup_goal[1], robot_id, 0, true, r_sup_goal[0], reachable);
        if (reachable) {
            log("Found stable support pre pose for task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::DEBUG);
            return true;
        }
        else {
            log("No nearby pre-support pose found for  task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::WARN);
            return false;
        }
        return false;
    }

    bool checkStability(int press_pt_x, int press_pt_y, int press_pt_z, bool use_support, 
            int support_x, int support_y, int support_z, int press_ori, int task_idx) {
        // check if the support pose is stable
        
        auto tic = std::chrono::high_resolution_clock::now();

        apex_mr::StabilityScore srv;
        // build a new json of lego structure and save the lego structure
        Json::Value lego_structure;
        
        vec<std::string> brick_names = lego_ptr_->get_brick_names();

        int index = 1;
        for (int i = 0; i < task_idx; i++) {
            auto cur_graph_node = task_json_[std::to_string(i+1)]; 

            Json::Value brick;
            brick["brick_id"] = cur_graph_node["brick_id"].asInt();
            brick["x"] = cur_graph_node["x"].asInt();
            brick["y"] = cur_graph_node["y"].asInt();
            brick["z"] = cur_graph_node["z"].asInt();
            brick["ori"] = cur_graph_node["ori"].asInt();
            lego_structure[std::to_string(index++)] = brick;
        }

        for (int i = 0; i < brick_names.size(); i++) {
            std::string brick_name = brick_names[i];
            int x, y, z, h, w, ori;
            lego_ptr_->get_init_brick_xyzo(brick_name, x, y, z, ori);
            auto dash_id = brick_name.find("_");
            std::string brick_id = brick_name.substr(1, dash_id - 1);

            Json::Value brick;
            brick["brick_id"] = std::stoi(brick_id);
            brick["x"] = x;
            brick["y"] = y;
            brick["z"] = z;
            brick["ori"] = ori;
            lego_structure[std::to_string(index++)] = brick;
        }

        Json::Value press_brick;
        press_brick["brick_id"] = 90;
        press_brick["x"] = press_pt_x;
        press_brick["y"] = press_pt_y;
        press_brick["z"] = press_pt_z;
        press_brick["ori"] = press_ori;
        lego_structure[std::to_string(index++)] = press_brick;

        std::string lego_structure_path = output_dir_ + "/lego_structure.json";
        std::ofstream lego_structure_file(lego_structure_path);
        lego_structure_file << lego_structure;
        lego_structure_file.close();

        srv.request.lego_structure_path = lego_structure_path;
        srv.request.use_support = use_support;
        srv.request.support_x = support_x;
        srv.request.support_y = support_y;
        srv.request.support_z = support_z;
        srv.request.support_ori = 0;

        // Call the service
        bool stable = false;
        if (stability_score_client_.call(srv)) {
            // Process the response
            stable = srv.response.stable;
            
        } else {
            ROS_ERROR("Failed to call service stability_score_service");
        }
        auto toc = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() / 1000.0;
        stability_check_time_ += duration;

        return stable;
    }

    bool computeAssignment() {
        apex_mr::TaskAssignment srv;
        srv.request.output_dir = output_dir_;
        srv.request.output_fname = task_name_ + "_seq.json";
        srv.request.task_config_path = task_fname_;
        
        Eigen::MatrixXd record(100000, 28);
        int record_cnt = 0;

        auto tic = std::chrono::high_resolution_clock::now();
        if (assignment_client_.call(srv)) {
           if (srv.response.feasible) {
                assign_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tic).count() / 1000.0;

                std::ifstream task_file(output_dir_ + "/" + srv.request.output_fname, std::ifstream::binary);
                task_file >> task_json_;
                num_tasks_ = task_json_.size();
                
                vec<std::string> brick_names = lego_ptr_->get_active_bricks_names();

                // save the corresponding poses to file
                for (int j = 0; j < num_tasks_; j++) {
                    auto cur_graph_node = task_json_[std::to_string(j+1)];
                    int brick_id = cur_graph_node["brick_id"].asInt();
                    std::string brick_seq = cur_graph_node["brick_seq"].asString();
                    std::string name = "b" + std::to_string(brick_id) + "_" + brick_seq;
                    if (name.find(".") != std::string::npos) {
                        name = name.substr(0, name.find("."));
                    }

                    int id = std::find(brick_names.begin(), brick_names.end(), name) - brick_names.begin();
 
                    int robot_id = cur_graph_node["robot_id"].asInt();
                    int press_side = cur_graph_node["press_side"].asInt();
                    int press_offset = cur_graph_node["press_offset"].asInt();
                    int g = (press_side - 1) * num_offsets_ + press_offset;
                    int sup_robot_id = cur_graph_node["sup_robot_id"].asInt();
                    int manip_type = 0;
                    if (cur_graph_node.isMember("manipulate_type")) {
                        manip_type = cur_graph_node["manipulate_type"].asInt();
                    }
                    log("Task " + std::to_string(j + 1) + " robot " + std::to_string(robot_id) + " brick name " + name
                        + " press side " + std::to_string(press_side) 
                        + " offset " + std::to_string(press_offset) + " support robot " + std::to_string(sup_robot_id) 
                        + " manip type " + std::to_string(manip_type), LogLevel::INFO);

                    // get the poses 
                    vecgoal pick_goals, handover_goals;
                    int pick_robot_id;
                    if (manip_type == 0) {
                        pick_robot_id = robot_id;
                        pick_goals = robot_id == 1 ? r1_lego_goals[id][g] : r2_lego_goals[id][g];
                    }
                    else {
                        pick_robot_id = robot_id == 1 ? 2 : 1;
                        pick_goals = robot_id == 1 ? r2_lego_goals[id][g] : r1_lego_goals[id][g];
                    }
                    vecgoal place_goals = robot_id == 1 ? r1_task_goals[j][g] : r2_task_goals[j][g];
                    vecgoal support_goals = sup_robot_id == 1 ? r1_support_goals[j][g] : r2_support_goals[j][g];

                    lego_manipulation::math::VectorJd r1_goal, r2_goal;
                    lego_manipulation::math::VectorJd home_qjd = static_cast<lego_manipulation::math::VectorJd>(home_q);
                    lego_manipulation::math::VectorJd receive_qjd = static_cast<lego_manipulation::math::VectorJd>(receive_q);
                    lego_manipulation::math::VectorJd home_handover_qjd = static_cast<lego_manipulation::math::VectorJd>(home_handover_q);
                    lego_manipulation::math::VectorJd home_receive_qjd = static_cast<lego_manipulation::math::VectorJd>(home_receive_q);
                    int use_r1, use_r2;
                    if (j == 0) {
                        writeRecordRow(record, record_cnt, 1, 1, 0, home_qjd, home_qjd);
                    }
                    if (manip_type == 0) {
                        
                        for (int mode = 1; mode <= 16; mode++) {
                            if (mode == 6 || mode == 16) {
                                r1_goal = home_qjd;
                                r2_goal = home_qjd;
                                use_r1 = 1;
                                use_r2 = 1;
                            }
                            else if (mode == 1) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? pick_goals[0] : home_qjd;
                                r2_goal = (robot_id == 2) ? pick_goals[0] : home_qjd;
                            }
                            else if (mode == 2) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? pick_goals[1] : home_qjd;
                                r2_goal = (robot_id == 2) ? pick_goals[1] : home_qjd;
                            }
                            else if (mode == 3) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? pick_goals[2] : home_qjd;
                                r2_goal = (robot_id == 2) ? pick_goals[2] : home_qjd;
                            }
                            else if (mode == 4) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? pick_goals[3] : home_qjd;
                                r2_goal = (robot_id == 2) ? pick_goals[3] : home_qjd;
                            }
                            else if (mode == 5) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? pick_goals[4] : home_qjd;
                                r2_goal = (robot_id == 2) ? pick_goals[4] : home_qjd;
                            }
                            else if (mode == 7) {
                                use_r1 = ((sup_robot_id == 1) ? 1 : 0);
                                use_r2 = ((sup_robot_id == 2) ? 1 : 0);
                                r1_goal = ((sup_robot_id == 1) ? support_goals[0] : home_qjd);
                                r2_goal = ((sup_robot_id == 2) ? support_goals[0] : home_qjd);
                            }
                            else if (mode == 8) {
                                use_r1 = ((sup_robot_id == 1) ? 1 : 0);
                                use_r2 = ((sup_robot_id == 2) ? 1 : 0);
                                r1_goal = ((sup_robot_id == 1) ? support_goals[1] : home_qjd);
                                r2_goal = ((sup_robot_id == 2) ? support_goals[1] : home_qjd);
                            }
                            else if (mode == 9) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[0] : ((sup_robot_id == 1) ? support_goals[1] : home_qjd);
                                r2_goal = (robot_id == 2) ? place_goals[0] : ((sup_robot_id == 2) ? support_goals[1] : home_qjd);
                            }
                            else if (mode == 10) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[1] : ((sup_robot_id == 1) ? support_goals[1] : home_qjd);
                                r2_goal = (robot_id == 2) ? place_goals[1] : ((sup_robot_id == 2) ? support_goals[1] : home_qjd);
                            }
                            else if (mode == 11) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[2] : ((sup_robot_id == 1) ? support_goals[0] : home_qjd);
                                r2_goal = (robot_id == 2) ? place_goals[2] : ((sup_robot_id == 2) ? support_goals[0] : home_qjd);
                            }
                            else if (mode == 12) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[3] : ((sup_robot_id == 1) ? support_goals[1] : home_qjd);
                                r2_goal = (robot_id == 2) ? place_goals[3] : ((sup_robot_id == 2) ? support_goals[1] : home_qjd);
                            }
                            else if (mode == 13) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[4] : ((sup_robot_id == 1) ? support_goals[0] : home_qjd);
                                r2_goal = (robot_id == 2) ? place_goals[4] : ((sup_robot_id == 2) ? support_goals[0] : home_qjd);
                            }
                            else if (mode == 14) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? home_qjd : ((sup_robot_id == 1) ? support_goals[0] : home_qjd);
                                r2_goal = (robot_id == 2) ? home_qjd : ((sup_robot_id == 2) ? support_goals[0] : home_qjd);
                            }
                            else if (mode == 15) {
                                use_r1 = ((sup_robot_id == 1) ? 1 : 0);
                                use_r2 = ((sup_robot_id == 2) ? 1 : 0);
                                r1_goal = ((sup_robot_id == 1) ? support_goals[0] : home_qjd);
                                r2_goal = ((sup_robot_id == 2) ? support_goals[0] : home_qjd);
                            }
                            

                            writeRecordRow(record, record_cnt, use_r1, use_r2, mode, r1_goal, r2_goal);
                            
                            // std::cout << "mode " << mode << std::endl;
                            // visualize_robot_pose(r1_goal, 0);
                            // visualize_robot_pose(r2_goal, 1);
                            // ros::Duration(0.5).sleep();
                        }
                    }
                    else {
                        for (int mode = 17; mode <= 43; mode ++) {
                            if (mode == 17) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_qjd : pick_goals[0];
                                r2_goal = (robot_id == 2) ? home_qjd : pick_goals[0];
                            }
                            else if (mode == 18) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_qjd : pick_goals[1];
                                r2_goal = (robot_id == 2) ? home_qjd : pick_goals[1];
                            }
                            else if (mode == 19) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_qjd : pick_goals[2];
                                r2_goal = (robot_id == 2) ? home_qjd : pick_goals[2];
                            }
                            else if (mode == 20) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_qjd : pick_goals[3];
                                r2_goal = (robot_id == 2) ? home_qjd : pick_goals[3];
                            }
                            else if (mode == 21) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_qjd : pick_goals[4];
                                r2_goal = (robot_id == 2) ? home_qjd : pick_goals[4];
                            }
                            else if (mode == 22) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_qjd : home_qjd;
                                r2_goal = (robot_id == 2) ? home_qjd : home_qjd;
                            }
                            else if (mode == 23) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? home_receive_qjd : home_qjd;
                                r2_goal = (robot_id == 2) ? home_receive_qjd : home_qjd;
                            }
                            else if (mode == 24) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_receive_qjd : home_handover_qjd;
                                r2_goal = (robot_id == 2) ? home_receive_qjd : home_handover_qjd;
                            }
                            else if (mode == 25) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? receive_qjd : home_handover_qjd;
                                r2_goal = (robot_id == 2) ? receive_qjd : home_handover_qjd;
                            }
                            else if (mode == 26) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? receive_qjd : r1_handover_goals[0];
                                r2_goal = (robot_id == 2) ? receive_qjd : r2_handover_goals[0];
                            }
                            else if (mode == 27) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? receive_qjd : r1_handover_goals[1];
                                r2_goal = (robot_id == 2) ? receive_qjd : r2_handover_goals[1];
                            }
                            else if (mode == 28) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? receive_qjd : r1_handover_goals[2];
                                r2_goal = (robot_id == 2) ? receive_qjd : r2_handover_goals[2];
                            }
                            else if (mode == 29) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? receive_qjd : r1_handover_goals[3];
                                r2_goal = (robot_id == 2) ? receive_qjd : r2_handover_goals[3];
                            }
                            else if (mode == 30) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? receive_qjd : home_handover_qjd;
                                r2_goal = (robot_id == 2) ? receive_qjd : home_handover_qjd;
                            }
                            else if (mode == 31) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? receive_qjd : home_qjd;
                                r2_goal = (robot_id == 2) ? receive_qjd : home_qjd;
                            }
                            else if (mode == 32) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? home_receive_qjd : home_qjd;
                                r2_goal = (robot_id == 2) ? home_receive_qjd : home_qjd;
                            }
                            else if (mode == 33) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[0] : home_qjd;
                                r2_goal = (robot_id == 2) ? place_goals[0] : home_qjd;
                            }
                            else if (mode == 34) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[1] : home_qjd;
                                r2_goal = (robot_id == 2) ? place_goals[1] : home_qjd;
                            }
                            else if (mode == 35) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? place_goals[1] : support_goals[0];
                                r2_goal = (robot_id == 2) ? place_goals[1] : support_goals[0];
                            }
                            else if (mode == 36) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? place_goals[1] : support_goals[1];
                                r2_goal = (robot_id == 2) ? place_goals[1] : support_goals[1];
                            }
                            else if (mode == 37) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[2] : support_goals[1];
                                r2_goal = (robot_id == 2) ? place_goals[2] : support_goals[1];
                            }
                            else if (mode == 38) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[3] : support_goals[1];
                                r2_goal = (robot_id == 2) ? place_goals[3] : support_goals[1];
                            }
                            else if (mode == 39) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[4] : support_goals[1];
                                r2_goal = (robot_id == 2) ? place_goals[4] : support_goals[1];
                            }
                            else if (mode == 40) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? place_goals[5] : support_goals[1];
                                r2_goal = (robot_id == 2) ? place_goals[5] : support_goals[1];
                            }
                            else if (mode == 41) {
                                use_r1 = (robot_id == 1) ? 1 : 0;
                                use_r2 = (robot_id == 2) ? 1 : 0;
                                r1_goal = (robot_id == 1) ? home_qjd : support_goals[1];
                                r2_goal = (robot_id == 2) ? home_qjd : support_goals[1];
                            }
                            else if (mode == 42) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_qjd : support_goals[0];
                                r2_goal = (robot_id == 2) ? home_qjd : support_goals[0];
                            }
                            else if (mode == 43) {
                                use_r1 = (robot_id == 1) ? 0 : 1;
                                use_r2 = (robot_id == 2) ? 0 : 1;
                                r1_goal = (robot_id == 1) ? home_qjd : home_qjd;
                                r2_goal = (robot_id == 2) ? home_qjd : home_qjd;
                            }

                            writeRecordRow(record, record_cnt, use_r1, use_r2, mode, r1_goal, r2_goal);
                            
                            //visualize_robot_pose(r1_goal, 0);
                            //visualize_robot_pose(r2_goal, 1);
                        }
                    }
                }

                // write the record to file
                lego_manipulation::io::SaveMatToFile(record.block(0, 0, record_cnt, 28), output_dir_ + "/" + task_name_ + "_steps.csv");
                log("Saved task assignment to " + output_dir_ + "/" + task_name_ + "_steps.csv", LogLevel::INFO);
                
                return true;

            } else {
                ROS_ERROR("Assignment is not feasible");
                return false;
            }

        } else {
            ROS_ERROR("Failed to call service assignment_service");
            return false;
        }
    }

    void writeRecordRow(Eigen::MatrixXd &record, int &record_cnt, int use_r1, int use_r2, int mode,
            const lego_manipulation::math::VectorJd &r1_goal, const lego_manipulation::math::VectorJd &r2_goal) {
        
        // TODO: output fk based on mode (select fk_mode)
        Eigen::MatrixXd r1_T = lego_manipulation::math::FK(r1_goal, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), false);
        Eigen::MatrixXd r2_T = lego_manipulation::math::FK(r2_goal, lego_ptr_->robot_DH_r2(), lego_ptr_->robot_base_r2(), false);
        
        Eigen::Matrix3d r1_goal_rot = r1_T.block(0, 0, 3, 3);
        Eigen::Quaterniond r1_quat(r1_goal_rot);

        Eigen::Matrix3d r2_goal_rot = r2_T.block(0, 0, 3, 3);
        Eigen::Quaterniond r2_quat(r2_goal_rot);

        record.row(record_cnt) << use_r1, r1_T(0, 3), r1_T(1, 3), r1_T(2, 3), r1_quat.x(), r1_quat.y(), r1_quat.z(), r1_quat.w(),
                                            r1_goal(0), r1_goal(1), r1_goal(2), r1_goal(3), r1_goal(4), r1_goal(5),
                                use_r2, r2_T(0, 3), r2_T(1, 3), r2_T(2, 3), r2_quat.x(), r2_quat.y(), r2_quat.z(), r2_quat.w(),
                                            r2_goal(0), r2_goal(1), r2_goal(2), r2_goal(3), r2_goal(4), r2_goal(5);
        record_cnt++;
    }

    double getStbilityCheckTime() {
        return stability_check_time_;
    }

    double getIKReachabilityTime() {
        return ik_reachability_time_;
    }

    double getPlanningTime() {
        return planning_time_;
    }

    double getAssignTime() {
        return assign_time_;
    }

private:
    ros::NodeHandle nh_;
    
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr kinematic_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene::PlanningScenePtr planning_scene_;

    // lego pointer
    lego_manipulation::lego::Lego::Ptr lego_ptr_;
    Json::Value task_json_;
    std::string task_fname_;
    int num_tasks_ = 0;
    int num_offsets_ = 7;
    double twist_rad_, handover_twist_rad_;

    // update gazebo state
    ros::ServiceClient set_state_client_;
    ros::ServiceClient planning_scene_diff_client;

    // cost matrix stuff
    Eigen::MatrixXd home_q, home_handover_q, home_receive_q, receive_q;
    Eigen::Matrix4d y_n90, y_s90, z_180;
    Eigen::MatrixXd twist_R_pick, twist_R_place;
    lego_manipulation::math::VectorJd pick_offset;
    double sup_down_offset = -0.001;
    ros::ServiceClient stability_score_client_;
    ros::ServiceClient assignment_client_;
    vec3d<std::string> world_grid_;
    bool motion_plan_cost_;
    bool check_stability_;
    bool print_debug_;
    bool optimize_poses_;
    bool IK_status_;

    // lego poses computed during cost matrix
    vec2d<vecgoal> r1_lego_goals, r2_lego_goals;
    vec2d<vecgoal> r1_task_goals, r2_task_goals;
    vec2d<vecgoal> r1_support_goals, r2_support_goals;
    vecgoal r1_handover_goals, r2_handover_goals;

    std::vector<std::string> group_names_; // group name for moveit controller services
    std::vector<std::string> eof_names_;
    std::shared_ptr<MoveitInstance> instance_;
    std::string output_dir_; 
    std::string task_name_;

    // statistics
    double stability_check_time_ = 0.0;
    double ik_reachability_time_ = 0.0;
    double planning_time_ = 0.0;
    double assign_time_ = 0.0;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "lego_node");
    
    // start ros node
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Read ROS Params
    std::string config_fname, task_fname, root_pwd, output_dir, task_name;
    bool motion_plan_cost, check_stability, optimize_poses;
    bool print_debug;

    std::vector<std::string> group_names = {"left_arm", "right_arm"};
    for (int i = 0; i < 2; i++) {
        if (nh.hasParam("group_name_" + std::to_string(i))) {
            nh.getParam("group_name_" + std::to_string(i), group_names[i]);
       }
    }
    std::vector<std::string> eof_links = {"left_arm_link_tool", "right_arm_link_tool"};

    nh.param<std::string>("config_fname", config_fname, "");
    nh.param<std::string>("task_fname", task_fname, "");
    nh.param<std::string>("task_name", task_name, "");
    nh.param<std::string>("root_pwd", root_pwd, "");
    nh.param<std::string>("output_dir", output_dir, "");
    nh.param<bool>("motion_plan_cost", motion_plan_cost, false);
    nh.param<bool>("check_stability", check_stability, true);
    nh.param<bool>("print_debug", print_debug, false);
    nh.param<bool>("optimize_poses", optimize_poses, true);
    
    // Initialize the Dual Arm Planner
    if (print_debug) {
        setLogLevel(LogLevel::DEBUG);
    }
    else {
        setLogLevel(LogLevel::INFO);
    }

    TaskAssignment planner(output_dir, task_name, group_names, eof_links, motion_plan_cost, 
        check_stability, optimize_poses, print_debug);

    ROS_INFO("Setting up the Lego Factory");

    // Read the lego poses
    if (!config_fname.empty() && !root_pwd.empty()) {
        planner.setLegoFactory(config_fname, root_pwd, task_fname);
        planner.initLegoPositions();
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    bool success = planner.calculateCostMatrix();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.0;

    if (success) {
        success &= planner.computeAssignment();
    }

    if (success) {
        ROS_INFO("Time taken to calculate cost matrix: %f s", duration);
        ROS_INFO("Time taken to check stability: %f s", planner.getStbilityCheckTime());
        ROS_INFO("Time taken to compute and check IK reachability: %f s", planner.getIKReachabilityTime());
        ROS_INFO("Time taken to compute task assignment: %f s", planner.getAssignTime());
        ROS_INFO("Time taken to plan: %f s", planner.getPlanningTime());
        double total_time = duration + planner.getAssignTime() ;
        ROS_INFO("Total time taken: %f s", total_time);
        ROS_INFO("Task Assignment is feasible");
    } else {
        ROS_ERROR("Task Assignment is not feasible");
    }

    ros::shutdown();
    return 0;
}
