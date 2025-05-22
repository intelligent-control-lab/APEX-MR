#include <fstream>
#include <iostream>
#include <sys/resource.h>
#include <ctime>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <planner.h>
#include <shortcutter.h>
#include <adg.h>
#include <logger.h>
#include <lego/Lego.hpp>
#include <lego_policy.h>

const std::string PLANNING_GROUP = "dual_arms";

struct GoalPose {
    bool use_robot;
    geometry_msgs::PoseStamped pose;
    std::vector<double> joint_values;
};

bool parsePoseLine(const std::string& line, std::vector<GoalPose>& poses) {
    std::istringstream stream(line);
    GoalPose pose1, pose2;
    pose1.joint_values.resize(7, 0);
    pose2.joint_values.resize(7, 0);

    // Read use_robot flags and pose data for each robot
    // line format: use_robot1,x,y,z,qx,qy,qz,qw,use_robot2,x,y,z,qx,qy,qz,qw
    char c;
    stream >> pose1.use_robot >> c >> pose1.pose.pose.position.x >> c >> pose1.pose.pose.position.y >> c >> pose1.pose.pose.position.z >> c
         >> pose1.pose.pose.orientation.x >> c >> pose1.pose.pose.orientation.y >> c >> pose1.pose.pose.orientation.z >> c >> pose1.pose.pose.orientation.w >> c 
         >> pose1.joint_values[0] >> c >> pose1.joint_values[1] >> c >> pose1.joint_values[2] >> c >> pose1.joint_values[3] >> c >> pose1.joint_values[4] >> c >> pose1.joint_values[5] >> c
         >> pose2.use_robot >> c >> pose2.pose.pose.position.x >> c >> pose2.pose.pose.position.y >> c >> pose2.pose.pose.position.z >> c 
         >> pose2.pose.pose.orientation.x >> c >> pose2.pose.pose.orientation.y >> c >> pose2.pose.pose.orientation.z >> c >> pose2.pose.pose.orientation.w >> c
         >> pose2.joint_values[0] >> c >> pose2.joint_values[1] >> c >> pose2.joint_values[2] >> c >> pose2.joint_values[3] >> c >> pose2.joint_values[4] >> c >> pose2.joint_values[5];

    // Check for stream errors
    if (stream.fail()) {
        ROS_ERROR("Failed to parse line: %s", line.c_str());
        return false;
    }
    pose1.pose.header.frame_id = "world";
    pose2.pose.header.frame_id = "world";
    for (size_t i = 0; i < 6; i++) {
        pose1.joint_values[i] = pose1.joint_values[i] * M_PI / 180.0;
        pose2.joint_values[i] = pose2.joint_values[i] * M_PI / 180.0;
    }

    poses.push_back(pose1);
    poses.push_back(pose2);

    return true;
}

void readPosesFromFile(const std::string& file_path, std::vector<std::vector<GoalPose>>& all_poses) {
    std::ifstream file(file_path);
    std::string line;
    if (!file.is_open()) {
        ROS_ERROR("Could not open file: %s", file_path.c_str());
        return;
    }

    while (getline(file, line)) {
        std::vector<GoalPose> poses;
        if (parsePoseLine(line, poses)) {
            all_poses.push_back(poses);
        }
    }

    file.close();
}

class DualArmPlanner {
public:
    DualArmPlanner(const std::string &planner_type, const std::string &output_dir,
                const std::vector<std::string> &group_names, bool async, bool mfi, bool fake_move, double vmax) : 
        planner_type_(planner_type), output_dir_(output_dir), group_names_(group_names), async_(async), mfi_(mfi), fake_move_(fake_move) {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model_ = robot_model_loader.getModel();
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        kinematic_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
        kinematic_state_->setToDefaultValues();
         // Create the planning scene from robot model
        // Planning scene monitor.
        // planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        // planning_scene_monitor_->startSceneMonitor();
        // planning_scene_monitor_->startStateMonitor();
        // planning_scene_monitor_->startWorldGeometryMonitor();
        // planning_scene_monitor_->requestPlanningSceneState();
        ros::Duration(0.3).sleep();
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

        planning_scene_diff_client = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();

        instance_ = std::make_shared<MoveitInstance>(kinematic_state_, move_group_->getName(), planning_scene_);
        instance_->setNumberOfRobots(2);
        instance_->setRobotNames({"left_arm", "right_arm"});
        instance_->setRobotDOF(0, 7);
        instance_->setRobotDOF(1, 7);
        instance_->setVmax(vmax);
        instance_->setPlanningSceneDiffClient(planning_scene_diff_client);

    }

    bool solveIKJointlyForPose(const EigenSTL::vector_Isometry3d &poses, 
                                            const std::string& group_name,
                                            const std::vector<std::string>& eef_links,
                                            std::vector<double>& joint_values) {
        
        // Function to check state validity, considering collisions
        moveit::core::GroupStateValidityCallbackFn stateValidityCheckFn = [this](robot_state::RobotState* robot_state, 
                                                    const robot_state::JointModelGroup* jm_group, 
                                                    const double* joint_group_variable_values)->bool {
            robot_state->setJointGroupPositions(jm_group, joint_group_variable_values);
            robot_state->update();
            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            collision_request.group_name = jm_group->getName();
            planning_scene_->checkCollision(collision_request, collision_result, *robot_state);
            return !collision_result.collision;
        };

        const robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
        std::vector<std::vector<double>> consistency_limits;
        
        kinematic_state_->setToDefaultValues();
        bool found_ik = kinematic_state_->setFromIKSubgroups(joint_model_group,
                            poses, eef_links, consistency_limits, 
                            0.0, // timeout
                            stateValidityCheckFn);
        if (found_ik) {
            kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
            return true;
        } else {
            ROS_ERROR("Failed to find IK solution for end effectors");
            return false;
        }
    }

    bool solveIKForPose(const geometry_msgs::PoseStamped& pose, const std::string& group_name, const std::string& eef_link, std::vector<double>& joint_values) {
        const robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
        auto ik_link = robot_model_->getLinkModel(eef_link);
        if (!ik_link) {
            ROS_ERROR("Could not find end effector link: %s", eef_link.c_str());
            return false;
        }

        // Convert Pose to Eigen
        Eigen::Isometry3d target_pose_eigen;
        tf2:fromMsg(pose.pose, target_pose_eigen);

        kinematic_state_->setToDefaultValues();
        bool found_ik = kinematic_state_->setFromIK(joint_model_group, target_pose_eigen, eef_link);

        if (found_ik) {
            kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
            return true;
        } else {
            ROS_ERROR("Failed to find IK solution for end effector: %s", eef_link.c_str());
            return false;
        }
    }


    bool planJointSpace(const std::vector<RobotPose>& goal_poses, const std::vector<double>& joint_values,
                MRTrajectory &solution) {
        bool success;
        if (planner_type_ == "BITstar") {
                move_group_->setPlannerId("BITstar");
                success = moveit_plan(joint_values, solution);
        } else if (planner_type_ == "SingleAgent") {
                move_group_->setPlanningTime(1.0);
                move_group_->setPlannerId("RRTConnect");
                success = moveit_plan(joint_values, solution);
        } else if (planner_type_ == "RRTConnect") {
            move_group_->setPlanningTime(1.0);
            move_group_->setPlannerId("RRTConnect");
            success = moveit_plan(joint_values, solution);
        }

        return success;
    }

    void setup_once() {
        /*
        Set joint name and record start locations. Necessary for execution
        */
        joint_names_ = move_group_->getVariableNames();
        joint_names_split_.clear();
        left_arm_joint_state_received = false;
        right_arm_joint_state_received = false;

        // create a directory for saving TPGs if it does not exist
        if (!boost::filesystem::exists(output_dir_)) {
            boost::filesystem::create_directories(output_dir_);
        }
        
        std::time_t t = std::time(nullptr);
        std::tm tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d%H%M%S");
        std::string timestamp = oss.str();
        policy_cfg_.exec_stats_file = output_dir_ + "/lego_policy_stats_" + timestamp + ".csv";

        if (mfi_) {
            std::vector<std::string> names;
            names.push_back("joint_1_s");
            names.push_back("joint_2_l");
            names.push_back("joint_3_u");
            names.push_back("joint_4_r");
            names.push_back("joint_5_b");
            names.push_back("joint_6_t");
            joint_names_split_.push_back(names);
            joint_names_split_.push_back(names);
            current_joints_.resize(14, 0.0);
           
            left_arm_sub = nh_.subscribe("/" + group_names_[0] + "/joint_states", 1, &DualArmPlanner::left_arm_joint_state_cb, this);
            right_arm_sub = nh_.subscribe("/" + group_names_[1] + "/joint_states", 1, &DualArmPlanner::right_arm_joint_state_cb, this);

        }
        else {
            current_joints_.resize(14, 0.0);
            for (int i = 0; i < 2; i++) {
                std::vector<std::string> name_i;
                for (size_t j = 0; j < 7; j++) {
                    name_i.push_back(joint_names_[i*7+j]);
                }
                joint_names_split_.push_back(name_i);
            }
            dual_arm_sub = nh_.subscribe("joint_states", 1, &DualArmPlanner::dual_arm_joint_state_cb, this);
        }

        // Get all sub-planning groups
        const std::vector<std::string>& group_names = robot_model_->getJointModelGroupNames();

        for (const std::string& group_name : group_names)
        {
            if (group_name == PLANNING_GROUP) {
                continue;
            }
            // Get the JointModelGroup
            const robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
            if (!joint_model_group)
            {
                ROS_WARN_STREAM("Failed to get JointModelGroup for group: " << group_name);
                continue;
            }

            // Get link names for the group
            const std::vector<std::string>& link_names = joint_model_group->getLinkModelNames();
            robot_links_.push_back(link_names);

            log("Planning Group: " + group_name, LogLevel::INFO);
            log("Link Names:", LogLevel::INFO);

            for (const std::string& link_name : link_names)
            {
                log("  - " + link_name, LogLevel::INFO);
            }
        }
    }

    bool moveit_plan(const std::vector<double>& joint_values, MRTrajectory& solution) {
        auto start_state = planning_scene_->getCurrentStateNonConst();
        start_state.setJointGroupPositions(move_group_->getName(), current_joints_);
        move_group_->setStartState(start_state);
        move_group_->setJointValueTarget(joint_values);
        move_group_->setPlanningTime(planning_time_limit_);
        ROS_INFO("Planning with planner: %s", move_group_->getPlannerId().c_str());
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            ROS_INFO("Planning succeeded");
            convertSolution(instance_, my_plan.trajectory_, solution);
            // set the new start state to the target state
        } else {
            ROS_ERROR("Planning failed");
        }

        return success;
    }

    bool segment_plan_rrt(int robot_id, std::shared_ptr<PlanInstance> inst, const std::vector<double> &goal_pose, RobotTrajectory &solution)
    {
        planning_interface::MotionPlanRequest req;
        req.group_name = group_names_[robot_id];
        req.allowed_planning_time = planning_time_limit_;
        req.planner_id = "RRTConnect";

        // Set the start state
        req.start_state.joint_state.name = joint_names_split_[robot_id];
        std::vector<double> start_pose(7, 0.0);
        for (size_t j = 0; j < 7; j++) {
            start_pose[j] = current_joints_[robot_id*7+j];
        }
        req.start_state.joint_state.position = start_pose;

        // add attached object
        // std::vector<Object> attached_objs = inst->getAttachedObjects(robot_id);
        // for (auto &obj : attached_objs) {
        //     moveit_msgs::AttachedCollisionObject co;
        //     co.link_name = obj.parent_link;
        //     co.object.id = obj.name;
        //     co.object.header.frame_id = obj.parent_link;
        //     co.object.operation = co.object.ADD;
        //     co.object.primitives.push_back(inst->getPrimitive(obj));
        //     co.object.primitive_poses.push_back(inst->getPose(obj));
        //     req.start_state.attached_collision_objects.push_back(co);
        // }

        // set the goal state
        moveit::core::RobotModelConstPtr robot_model = planning_scene_->getRobotModel();
        robot_state::RobotState goal_state(robot_model);
        const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group_names_[robot_id]);
        goal_state.setJointGroupPositions(joint_model_group, goal_pose);
        moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

        req.goal_constraints.clear();
        req.goal_constraints.push_back(joint_goal);

        planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, nh_, "planning_plugin", "request_adapters"));

        planning_interface::MotionPlanResponse res;
        planning_pipeline->generatePlan(planning_scene_, req, res);

        if (res.error_code_.val != res.error_code_.SUCCESS)
        {
            ROS_ERROR("Could not compute plan successfully");
            return false;
        }

        moveit_msgs::RobotTrajectory plan_traj;
        res.trajectory_->getRobotTrajectoryMsg(plan_traj);
        convertSolution(inst, plan_traj, robot_id, solution);
        return true;
    }


    void set_tpg(std::shared_ptr<tpg::TPG> tpg) {
        tpg_ = tpg;
    }

    void set_act_graph(std::shared_ptr<ActivityGraph> act_graph) {
        act_graph_ = act_graph;
    }

    void reset_joint_states_flag() {
        left_arm_joint_state_received = false;
        right_arm_joint_state_received = false;
        while (!left_arm_joint_state_received || !right_arm_joint_state_received) {
            ros::Duration(0.01).sleep();
        }
    }

    bool fake_execute(const MRTrajectory & solution) {
        const RobotPose &pose = solution[0].trajectory.back();
        instance_->moveRobot(0, pose);
        instance_->updateScene();

        const RobotPose &pose2 = solution[1].trajectory.back();
        instance_->moveRobot(1, pose2);
        instance_->updateScene();

        for (size_t i = 0; i < 6; i++) {
            current_joints_[i] = pose.joint_values[i];
            current_joints_[7+i] = pose2.joint_values[i];
        }
        return true;
    }

    bool execute(std::shared_ptr<tpg::TPG> tpg) {
        
        bool success = true;
        if (async_) {
            std::vector<ros::ServiceClient> clients;
            for (auto group_name: group_names_) {
                clients.push_back(nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/" + group_name + "/yk_execute_trajectory"));
            }
            std::shared_ptr<ActivityGraph> act_graph = dynamic_cast<tpg::ADG*>(tpg.get())->getActGraph();
            std::shared_ptr<LegoPolicy> policy = std::make_shared<LegoPolicy>(lego_ptr_, group_names_, joint_names_split_, act_graph, policy_cfg_);
            dynamic_cast<tpg::ADG*>(tpg.get())->setPolicy(policy);

            // std::vector<std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> clients_action;
            // for (auto group_name: group_names_) {
            //     clients_action.push_back(std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(
            //         "/" + group_name + "/joint_trajectory_action", true));
            // }
            // policy->add_actionlib(clients_action);

#ifdef HAVE_YK_TASKS
            std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>> yk_clients;
            for (auto group_name: group_names_) {
                yk_clients.push_back(std::make_shared<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>("/" + group_name + "/yk_go_to_joints", true));
            }            
            policy->add_actionlib(yk_clients);

            std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToPoseAction>>> yk_goto_pose_clients;
            for (auto group_name: group_names_) {
                yk_goto_pose_clients.push_back(std::make_shared<actionlib::SimpleActionClient<yk_tasks::GoToPoseAction>>("/" + group_name + "/yk_go_to_pose", true));
            }
            policy->add_goto_pose_actionlib(yk_goto_pose_clients);

            std::vector<std::shared_ptr<ros::ServiceClient>> yk_stop_clients;
            for (auto group_name: group_names_) {
                yk_stop_clients.push_back(std::make_shared<ros::ServiceClient>(nh_.serviceClient<std_srvs::Trigger>("/" + group_name + "/yk_stop_trajectory", true)));
            }
            policy->add_stop_clients(yk_stop_clients);

            std::vector<std::shared_ptr<ros::ServiceClient>> yk_enable_clients;
            for (auto group_name: group_names_) {
                yk_enable_clients.push_back(std::make_shared<ros::ServiceClient>(nh_.serviceClient<std_srvs::Trigger>("/" + group_name + "/robot_enable", true)));
            }
            policy->add_enable_clients(yk_enable_clients);

            std::vector<std::shared_ptr<ros::ServiceClient>> yk_getpose_clients;
            for (auto group_name: group_names_) {
                yk_getpose_clients.push_back(std::make_shared<ros::ServiceClient>(nh_.serviceClient<yk_msgs::GetPose>("/" + group_name + "/yk_get_pose", true)));
            }
            policy->add_getpose_clients(yk_getpose_clients);

#endif

            success &= tpg->moveit_mt_execute(joint_names_split_, clients);
        } else {
            success &= tpg->moveit_execute(instance_, move_group_);
        }
        return success;
    }

    bool saveTPG(std::shared_ptr<tpg::TPG> tpg, const std::string &filename) {
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return false;
        }
        boost::archive::text_oarchive oa(ofs);
        oa << tpg;
        
        return true;
    }

    std::shared_ptr<tpg::TPG> loadTPG(const std::string &filename) {
        // open the file safely
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return nullptr;
        }
        std::shared_ptr<tpg::TPG> tpg = std::make_shared<tpg::TPG>();
        boost::archive::text_iarchive ia(ifs);
        ia >> tpg;
        return tpg;
    }

    void addTPGPlan(const ShortcutOptions &sc_options) {
        moveit_msgs::RobotTrajectory traj;
        traj.joint_trajectory.joint_names = joint_names_;
        double flowtime, makespan;
        tpg_->setSyncJointTrajectory(traj.joint_trajectory, flowtime, makespan);
        
        moveit_msgs::RobotTrajectory smoothed_traj;
        shortcutSolution(instance_, traj, smoothed_traj, sc_options);
        double new_makespan = smoothed_traj.joint_trajectory.points.back().time_from_start.toSec();

        logProgressFileAppend(sc_options.progress_file, "task" + std::to_string(plans_.size()), makespan, new_makespan);

        MRTrajectory tpg_solution;
        convertSolution(instance_, smoothed_traj, tpg_solution, false);
        plans_.push_back(tpg_solution);
    }

    bool planAndMove(const std::vector<RobotPose>& poses, const tpg::TPGConfig &tpg_config, bool sequential, int plan_robot_id, MRTrajectory &solution) {
        if (fake_move_) {
            dual_arm_sub.shutdown();
        }
        else {
            reset_joint_states_flag();
        }

        auto t_start = std::chrono::high_resolution_clock::now();
        std::vector<double> all_joints;
        std::vector<double> all_joints_given;
        for (auto pose : poses) {
            all_joints_given.insert(all_joints_given.end(), pose.joint_values.begin(), pose.joint_values.end());
        }
        bool success = true;
        solution.clear();
        if (!sequential) {
            success = planJointSpace(poses, all_joints_given, solution);
        }
        else {
            for (int i = 0; i < group_names_.size(); i++) {
                RobotTrajectory traj;
                traj.robot_id = i;
                if (i != plan_robot_id) {
                    RobotPose pose = instance_->initRobotPose(i);
                    pose.joint_values = poses[i].joint_values;
                    traj.trajectory = {pose};
                    traj.times = {0.0};
                    traj.cost = 0;
                }
                else {
                    success = segment_plan_rrt(i, instance_, poses[i].joint_values, traj);
                }
                solution.push_back(traj);
            }
        }
        planning_colcheck_ += instance_->numCollisionChecks();

        if (success) {
            int t_plan_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count();

            if (fake_move_) {
                success &= fake_execute(solution);
            } else {
                if (tpg_ == nullptr) {
                    tpg_ = std::make_shared<tpg::TPG>();
                }
                tpg_->reset();
                
                success &= tpg_->init(instance_, solution, tpg_config);
                tpg_colcheck_ += instance_->numCollisionChecks();
                success &= execute(tpg_);
            }
        }
        counter_++;
        return success;
    }

    bool sequential_plan(int start_task_idx, bool load_tpg, const std::vector<std::vector<GoalPose>> &all_poses,
                        const ShortcutOptions &sc_options, const tpg::TPGConfig &tpg_config,
                        bool plan_skeleton_only) {
        
        std::shared_ptr<ActivityGraph> act_graph = std::make_shared<ActivityGraph>(2);
        std::vector<std::shared_ptr<tpg::TPG>> tpgs;
        int task_idx = start_task_idx;
        int manip_type = getManipType(task_idx);
        int mode = 0;
        std::string brick_name;
        std::vector<std::string> bottom_bricks, side_bricks, top_bricks, sup_side_bricks;
        int robot_id = -1;
        int other_robot_id = -1;
        int sup_robot = -1;
        ActPtr last_drop_act, last_receive_act;
        std::vector<double> init_joints = getCurrentJoints();
        std::vector<double> r1_init_joints(init_joints.begin(), init_joints.begin() + 7);
        std::vector<double> r2_init_joints(init_joints.begin() + 7, init_joints.end());
        std::vector<Activity::Type> last_act_type = {Activity::Type::home, Activity::Type::home};
        ActPtr last_act = nullptr;

        // add table as object
        act_graph->add_obj(instance_->getObject("table"));

        int i = 0;
        while (task_idx <= num_tasks_) {
            // set the activity's start and goal pose
            std::vector<RobotPose> start_poses;
            RobotPose pose0 = instance_->initRobotPose(0);
            pose0.joint_values = r1_init_joints;
            start_poses.push_back(pose0);
            
            RobotPose pose1 = instance_->initRobotPose(1);
            pose1.joint_values = r2_init_joints;
            start_poses.push_back(pose1);
            
            // get brick name and whether it's form a feeding station
            getLegoBrickName(task_idx, brick_name);
            bool from_station = (brick_name.find("station") != std::string::npos);

            if (mode == 0) {
                act_graph->add_act(0, Activity::Type::home);
                act_graph->add_act(1, Activity::Type::home);
            }
            if (mode == 1) {
                robot_id = getRobot(task_idx);
                other_robot_id = (robot_id == 0) ? 1 : 0;
                sup_robot = getSupportRobot(task_idx);
                
                Object obj = getLegoStart(brick_name);
                ObjPtr obj_node = act_graph->add_obj(obj);
                if (from_station) {
                    obj_node->vanish = true;
                    addMoveitCollisionObject(brick_name);
                }

                act_graph->add_act(robot_id, Activity::Type::pick_tilt_up);
                act_graph->add_act(other_robot_id, Activity::Type::home);
                
            }
            if (mode == 2) {
                act_graph->add_act(robot_id, Activity::Type::pick_up);
                act_graph->add_act(other_robot_id, Activity::Type::home);
                act_graph->set_collision(brick_name, eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::pick_up), true);
            }
            if (mode == 3) {
                act_graph->add_act(robot_id, Activity::Type::pick_down);
                act_graph->add_act(other_robot_id, Activity::Type::home);
                getLegoBottom(brick_name, task_idx, false, bottom_bricks);
                for (const auto & bottom_brick : bottom_bricks) {
                    act_graph->set_collision(bottom_brick, eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::pick_down), true);
                    act_graph->set_collision(bottom_brick, brick_name, act_graph->get_last_act(robot_id, Activity::Type::pick_down), true);
                }
            }
            if (mode == 4) {
                ROS_INFO("attach lego to robot %d", robot_id);
                act_graph->add_act(robot_id, Activity::Type::pick_twist);
                act_graph->add_act(other_robot_id, Activity::Type::home);
                act_graph->attach_obj(act_graph->get_last_obj(brick_name), eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::pick_twist));
                
            }
            if (mode == 5) {
                act_graph->add_act(robot_id, Activity::Type::pick_twist_up);
                act_graph->add_act(other_robot_id, Activity::Type::home);
            }
            if (mode == 6) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(other_robot_id, Activity::Type::home);
                for (const auto & bottom_brick : bottom_bricks) {
                    act_graph->set_collision(bottom_brick, eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::home), false);
                }
            }
            if (mode == 7) {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1) {
                    act_graph->add_act(sup_robot, Activity::Type::support_pre);
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
            }
            if (mode == 8) {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1) {
                    auto sup_act = act_graph->add_act(sup_robot, Activity::Type::support);
                    if (last_drop_act != nullptr && last_drop_act->robot_id != sup_robot) {
                        act_graph->add_type2_dep(sup_act, last_drop_act);
                    }
                    getLegoSuppNearby(task_idx, sup_side_bricks);
                    for (const auto & sup_side_brick : sup_side_bricks) {
                        act_graph->set_collision(sup_side_brick, eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::support), true);
                    }
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
            } 
            if (mode == 9) {
                act_graph->add_act(robot_id, Activity::Type::drop_tilt_up);
                if (sup_robot > -1) {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
            }
            if (mode == 10) {
                act_graph->add_act(robot_id, Activity::Type::drop_up);
                if (sup_robot > -1) {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
            }
            if (mode == 11) {
                if (sup_robot > -1) {
                    act_graph->add_act(robot_id, Activity::Type::drop_down, act_graph->get_last_act(sup_robot, Activity::Type::support));
                    act_graph->add_act(sup_robot, Activity::Type::support);
                } else {
                    act_graph->add_act(robot_id, Activity::Type::drop_down);
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
                if (last_drop_act != nullptr && last_drop_act->robot_id != robot_id) {
                    act_graph->add_type2_dep(act_graph->get_last_act(robot_id, Activity::Type::drop_down), last_drop_act);
                }
                getLegoBottom(brick_name, task_idx, true, bottom_bricks);
                for (const auto & bottom_brick : bottom_bricks) {
                    act_graph->set_collision(bottom_brick, brick_name, act_graph->get_last_act(robot_id, Activity::Type::drop_down), true);
                } 
            }
            if (mode == 12) {
                ROS_INFO("detach lego from robot %d", robot_id);
                Object obj = getLegoTarget(task_idx);
                act_graph->add_obj(obj);
                act_graph->add_act(robot_id, Activity::Type::drop_twist);
                if (sup_robot > -1) {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
                act_graph->detach_obj(act_graph->get_last_obj(brick_name), act_graph->get_last_act(robot_id, Activity::Type::drop_twist));

                // disable collision between the block on the side of the twist with the tool
                getLegoTwistNext(task_idx, brick_name, side_bricks);
                for (const auto & side_brick : side_bricks) {
                    act_graph->set_collision(side_brick, eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::drop_twist), true);
                }
                
            }
            if (mode == 13) {
                last_drop_act = act_graph->add_act(robot_id, Activity::Type::drop_twist_up);
                if (sup_robot > -1) {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
                // reenable collision between the block on the side of the twist with the tool
                for (const auto & side_brick : side_bricks) {
                    act_graph->set_collision(side_brick, eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::drop_twist_up), false);
                }
            }
            if (mode == 14) {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1) {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
                act_graph->set_collision(brick_name, eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::home), false);
            }
            if (mode == 15) {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1) {
                    act_graph->add_act(sup_robot, Activity::Type::support_pre, act_graph->get_last_act(robot_id, Activity::Type::drop_twist));
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
            }
            if (mode == 16) {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1) {
                    act_graph->add_act(sup_robot, Activity::Type::home);
                    for (const auto & sup_side_brick : sup_side_bricks) {
                        act_graph->set_collision(sup_side_brick, eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::home), false);
                    }
                } else {
                    act_graph->add_act(other_robot_id, Activity::Type::home);
                }
            }
            if (mode == 17) {
                robot_id = getRobot(task_idx);
                other_robot_id = (robot_id == 0) ? 1 : 0;
                sup_robot = getSupportRobot(task_idx);

                Object obj = getLegoStart(brick_name);
                ObjPtr obj_node = act_graph->add_obj(obj);
                if (from_station) {
                    obj_node->vanish = true;
                    addMoveitCollisionObject(brick_name);
                }
                obj_node->handover = true;

                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_tilt_up);
            }
            if (mode == 18) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_up);
                act_graph->set_collision(brick_name, eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::pick_up), true);
            }
            if (mode == 19) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_down);
                getLegoBottom(brick_name, task_idx, false, bottom_bricks);
                for (const auto & bottom_brick : bottom_bricks) {
                    act_graph->set_collision(bottom_brick, eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::pick_down), true);
                    act_graph->set_collision(bottom_brick, brick_name, act_graph->get_last_act(sup_robot, Activity::Type::pick_down), true);
                }
            }
            if (mode == 20) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_twist);
                act_graph->attach_obj(act_graph->get_last_obj(brick_name), eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::pick_twist));                
            }
            if (mode == 21) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_twist_up);
            }
            if (mode == 22) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::home);
                for (const auto & bottom_brick : bottom_bricks) {
                    act_graph->set_collision(bottom_brick, eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::home), false);
                }
            }
            if (mode == 23) {
                act_graph->add_act(robot_id, Activity::Type::home_receive);
                act_graph->add_act(sup_robot, Activity::Type::home);
            }
            if (mode == 24) {
                act_graph->add_act(robot_id, Activity::Type::home_receive);
                act_graph->add_act(sup_robot, Activity::Type::home_handover);
            }
            if (mode == 25) {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::home_handover);
            }
            if (mode == 26) {
                last_receive_act = act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::handover_up);
            }
            if (mode == 27) {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::handover_down, last_receive_act);
                act_graph->set_collision(brick_name, eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::receive), true);
            }
            if (mode == 28) {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::handover_twist);
                Object obj = getLegoHandover(task_idx, start_poses[sup_robot]);
                ObjPtr objptr = act_graph->add_obj(obj);
                objptr->handover = true;
                act_graph->detach_obj(act_graph->get_last_obj(brick_name), act_graph->get_last_act(sup_robot, Activity::Type::handover_twist));
            }
            if (mode == 29) {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::handover_twist_up);
            }
            if (mode == 30) {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::home_handover);
                act_graph->set_collision(brick_name, eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::home_handover), false);
            }
            if (mode == 31) {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::home);
            }
            if (mode == 32) {
                act_graph->add_act(robot_id, Activity::Type::home_receive, act_graph->get_last_act(sup_robot, Activity::Type::handover_twist_up));
                act_graph->add_act(sup_robot, Activity::Type::home);
                act_graph->attach_obj(act_graph->get_last_obj(brick_name), eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::home_receive));
            }
            if (mode == 33) {
                act_graph->add_act(robot_id, Activity::Type::place_tilt_down_pre);
                act_graph->add_act(sup_robot, Activity::Type::home);
            }
            if (mode == 34) {
                act_graph->add_act(robot_id, Activity::Type::place_tilt_down);
                act_graph->add_act(sup_robot, Activity::Type::home);
            }
            if (mode == 35) {
                act_graph->add_act(robot_id, Activity::Type::place_tilt_down);
                act_graph->add_act(sup_robot, Activity::Type::press_up);
            }
            if (mode == 36) {
                act_graph->add_act(robot_id, Activity::Type::place_tilt_down);
                auto press_act = act_graph->add_act(sup_robot, Activity::Type::press_down);
                if (last_drop_act != nullptr && last_drop_act->robot_id != sup_robot) {
                    act_graph->add_type2_dep(press_act, last_drop_act);
                }
                getLegoTop(brick_name, task_idx, true, top_bricks);
                for (const auto & top_brick : top_bricks) {
                    act_graph->set_collision(top_brick, eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::press_down), true);
                }
            }
            if (mode == 37) {
                act_graph->add_act(robot_id, Activity::Type::place_down);
                act_graph->add_act(sup_robot, Activity::Type::press_down);
            }
            if (mode == 38) {
                act_graph->add_act(robot_id, Activity::Type::place_up, act_graph->get_last_act(sup_robot, Activity::Type::press_down));
                act_graph->add_act(sup_robot, Activity::Type::press_down);
                if (last_drop_act != nullptr && last_drop_act->robot_id != robot_id) {
                    act_graph->add_type2_dep(act_graph->get_last_act(robot_id, Activity::Type::place_up), last_drop_act);
                }
                for (const auto & top_brick : top_bricks) {
                    act_graph->set_collision(top_brick, brick_name, act_graph->get_last_act(robot_id, Activity::Type::place_up), true);
                }
            }
            if (mode == 39) {
                Object obj = getLegoTarget(task_idx);
                ObjPtr objptr = act_graph->add_obj(obj);
                objptr->handover = true;
                act_graph->add_act(robot_id, Activity::Type::place_twist);
                act_graph->add_act(sup_robot, Activity::Type::press_down);
                act_graph->detach_obj(act_graph->get_last_obj(brick_name), act_graph->get_last_act(robot_id, Activity::Type::place_twist));
            }
            if (mode == 40) {
                last_drop_act = act_graph->add_act(robot_id, Activity::Type::place_twist_down);
                act_graph->add_act(sup_robot, Activity::Type::press_down);
            }
            if (mode == 41) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::press_down);
                act_graph->set_collision(brick_name, eof_links_[robot_id], act_graph->get_last_act(robot_id, Activity::Type::home), false);
            }
            if (mode == 42) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::press_up, act_graph->get_last_act(robot_id, Activity::Type::place_twist));
            }
            if (mode == 43) {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::home);
                // renable collision with the press robot
                for (const auto & top_brick : top_bricks) {
                    act_graph->set_collision(top_brick, eof_links_[sup_robot], act_graph->get_last_act(sup_robot, Activity::Type::home), false);
                }
            }

            // set start and goal robot pose in task graph
            int plan_robot_id = -1;
            auto r1_act = act_graph->get_last_act(0);
            auto r2_act = act_graph->get_last_act(1);

            if (r1_act->type != last_act_type[0]) {
                plan_robot_id = 0;
            }
            
            if (r2_act->type != last_act_type[1]) {
                if (plan_robot_id != -1) {
                    log("There can only be one robot moving in sequential planning", LogLevel::ERROR);
                }
                plan_robot_id = 1;
            }

            r1_act->start_pose = pose0;
            pose0.joint_values = all_poses[i][0].joint_values;
            r1_act->end_pose = pose0;
            r1_init_joints = pose0.joint_values;
            last_act_type[0] = r1_act->type;
            
            r2_act->start_pose = pose1;
            pose1.joint_values = all_poses[i][1].joint_values;
            r2_act->end_pose = pose1;
            r2_init_joints = pose1.joint_values;
            last_act_type[1] = r2_act->type;

            ROS_INFO("mode %d, task_id: %d, robot_id: %d, sup_robot_id: %d, brick_name: %s", mode, task_idx, robot_id, sup_robot, brick_name.c_str());

            if (!plan_skeleton_only) {
                // update the scene
                for (const auto obj_node : r1_act->obj_attached) {
                    attachMoveitCollisionObject(obj_node->name(), 0, eof_links_[0], r1_act->start_pose);
                }
                for (const auto obj_node : r2_act->obj_attached) {
                    attachMoveitCollisionObject(obj_node->name(), 1, eof_links_[1], r2_act->start_pose);
                }
                for (const auto obj_node : r1_act->obj_detached) {
                    detachMoveitCollisionObject(obj_node->name(), r1_act->start_pose);
                }
                for (const auto obj_node : r2_act->obj_detached) {
                    detachMoveitCollisionObject(obj_node->name(), r2_act->start_pose);
                }
                for (const auto col_node : r1_act->collision_nodes) {
                    setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
                }
                for (const auto col_node : r2_act->collision_nodes) {
                    setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
                }

                auto tic = std::chrono::high_resolution_clock::now();
                MRTrajectory solution;
                if (load_tpg) {
                    std::shared_ptr<tpg::TPG> tpg = loadTPG(output_dir_ + "/tpg_" + std::to_string(i) + ".txt");
                    if (tpg != nullptr) {
                        set_tpg(tpg);
                        reset_joint_states_flag(); // do this after tpg is set
                        execute(tpg);
                    }
                }
                else {
                    bool success;
                    if (mode == 0) {
                        success = planAndMove({pose0, pose1}, tpg_config, false, plan_robot_id, solution);
                    }
                    else {
                        success = planAndMove({pose0, pose1}, tpg_config, true, plan_robot_id, solution);
                    }
                    if (!success) {
                        instance_->moveRobot(0, pose0);
                        instance_->updateScene();
                        ROS_ERROR("Failed to plan and move mode %d, task_id: %d, robot_id: %d, sup_robot_id: %d, brick_name: %s", mode, task_idx, robot_id, sup_robot, brick_name.c_str());
                        return false;
                    }
                    
                }
                
                double diff0 = solution[0].cost - instance_->computeDistance(solution[0].trajectory.front(), solution[0].trajectory.back()) / instance_->getVMax(0);
                double diff1 = solution[1].cost - instance_->computeDistance(solution[1].trajectory.front(), solution[1].trajectory.back()) / instance_->getVMax(1);

                MRTrajectory rediscretized_solution;
                rediscretizeSolution(instance_, solution, rediscretized_solution, tpg_config.dt);
                MRTrajectory smoothed_solution;
                Shortcutter shortcutter(instance_, sc_options);
                double makespan = shortcutter.calculate_makespan(rediscretized_solution);
                double new_makespan = makespan;
                if (diff0 > 0.1 || diff1 > 0.1) {
                    shortcutter.shortcutSolution(rediscretized_solution, smoothed_solution);
                    new_makespan = shortcutter.calculate_makespan(smoothed_solution);
                    log("Makespan reduced from " + std::to_string(makespan) + " to " + std::to_string(new_makespan), LogLevel::INFO);
                }
                else {
                    smoothed_solution = rediscretized_solution;
                }
                removeWait(instance_, smoothed_solution);
                auto toc = std::chrono::high_resolution_clock::now();
                double t_plan = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() * 0.001;
                planning_time_ += t_plan;

                std::string pose_str = "task" + std::to_string(plans_.size());
                logProgressFileAppend(sc_options.progress_file, pose_str, pose_str, t_plan, makespan, new_makespan);

                plans_.push_back(smoothed_solution);
                //addTPGPlan(sc_options);

                //saveTPG(tpg, output_dir_ + "/tpg_" + std::to_string(counter_) + ".txt");
                //tpg->saveToDotFile(output_dir_ + "/tpg_" + std::to_string(counter_) + ".dot");
            }

            if (mode == 0 && manip_type == 1) {
                mode = (manip_type == 0) ? 1 : 17;
            }
            else {
                mode ++;
                if (mode == 17 || mode == 44) {
                    task_idx ++;
                    manip_type = getManipType(task_idx);
                    mode = (manip_type == 0) ? 1 : 17;
                    ROS_INFO("New task_id: %d, manip_type: %d, mode %d", task_idx, manip_type, mode);
                }
            }
            i++;
        }
        
        act_graph->saveGraphToFile(output_dir_ + "/act_graph.txt");

        if (plan_skeleton_only) {
            set_act_graph(act_graph);
            return true;
        }
        else {
            ROS_INFO("Planning time total %f s %d collision checks", getPlanningTime(), getPlanningColCheck());
        
            // create adg
            MRTrajectory sync_solution;
            concatSyncSolution(instance_, act_graph, plans_, tpg_config.dt, sync_solution);
    
            auto adg = std::make_shared<tpg::ADG>(act_graph);
            bool init_succ = adg->init_from_asynctrajs(instance_, tpg_config, sync_solution);
            if (!init_succ) {
                adg->saveToDotFile(output_dir_ + "/adg.dot");
                return false;
            }
            set_tpg(adg);
            ROS_INFO("Adg init total %d collision checked ", getTPGColCheck() + instance_->numCollisionChecks());

            return true;
        }
    }

    bool sync_plan(int start_task_idx, bool load_tpg, const std::vector<std::vector<GoalPose>> &all_poses,
                        const ShortcutOptions &sc_options, const tpg::TPGConfig &tpg_config) {
        
        bool has_seq_plan = sequential_plan(start_task_idx, load_tpg, all_poses, sc_options, 
                        tpg_config, true);
        if (!has_seq_plan) {
            return false;
        }

        auto tic = std::chrono::high_resolution_clock::now();
        act_graph_->minimizeWait(instance_);
        auto toc = std::chrono::high_resolution_clock::now();
        double t_minimize = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() * 0.001;
        ROS_INFO("Minimize wait time %f s", t_minimize);
        act_graph_->saveGraphToFile(output_dir_ + "/act_graph_sync.txt");
        
        for (auto & obj_node : act_graph_->get_obj_nodes()) {
            if (obj_node->vanish) {
                instance_->removeObject(obj_node->name());
                instance_->updateScene();
            }
        }

        int max_act_num = std::max(act_graph_->num_activities(0), act_graph_->num_activities(1));
        for (int i = 0; i < max_act_num; i++) {
            ActPtr r1_act, r2_act;
            if (i < act_graph_->num_activities(0)) {
                r1_act = act_graph_->get(0, i);
            }
            else {
                r1_act = act_graph_->get_last_act(0);
            }
            if (i < act_graph_->num_activities(1)) {
                r2_act = act_graph_->get(1, i);
            }
            else {
                r2_act = act_graph_->get_last_act(1);
            }

            // update the scene
            for (const auto obj_node : r1_act->obj_attached) {
                if (obj_node->vanish) {
                    addMoveitCollisionObject(obj_node->name());
                }
                attachMoveitCollisionObject(obj_node->name(), 0, eof_links_[0], r1_act->start_pose);
            }
            for (const auto obj_node : r2_act->obj_attached) {
                if (obj_node->vanish) {
                    addMoveitCollisionObject(obj_node->name());
                }
                attachMoveitCollisionObject(obj_node->name(), 1, eof_links_[1], r2_act->start_pose);
            }
            for (const auto obj_node : r1_act->obj_detached) {
                detachMoveitCollisionObject(obj_node->name(), r1_act->start_pose);
            }
            for (const auto obj_node : r2_act->obj_detached) {
                detachMoveitCollisionObject(obj_node->name(), r2_act->start_pose);
            }
            for (const auto col_node : r1_act->collision_nodes) {
                setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
            }
            for (const auto col_node : r2_act->collision_nodes) {
                setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
            }

            RobotPose pose0 = r1_act->end_pose;
            RobotPose pose1 = r2_act->end_pose;

            auto tic = std::chrono::high_resolution_clock::now();
            MRTrajectory solution;  
            bool success = planAndMove({pose0, pose1}, tpg_config, false, -1, solution);
    
            if (!success) {
                ROS_ERROR("Failed to plan and move robot 0 task %d %s, robot 1 task %d %s", i, r1_act->type_string().c_str(), i, r2_act->type_string().c_str());
                return false;
            }
            ROS_INFO("Planned and move robot 0 task %d %s, robot 1 task %d %s", i, r1_act->type_string().c_str(), i, r2_act->type_string().c_str());


            double diff0 = solution[0].cost - instance_->computeDistance(solution[0].trajectory.front(), solution[0].trajectory.back()) / instance_->getVMax(0);
            double diff1 = solution[1].cost - instance_->computeDistance(solution[1].trajectory.front(), solution[1].trajectory.back()) / instance_->getVMax(1);

            MRTrajectory rediscretized_solution;
            rediscretizeSolution(instance_, solution, rediscretized_solution, tpg_config.dt);
            MRTrajectory smoothed_solution;
            Shortcutter shortcutter(instance_, sc_options);
            double makespan = shortcutter.calculate_makespan(rediscretized_solution);
            double flowtime = shortcutter.calculate_flowtime(rediscretized_solution);
            double new_makespan = makespan;
            double new_flowtime = flowtime;
            if (diff0 > 0.1 || diff1 > 0.1) {
                shortcutter.shortcutSolution(rediscretized_solution, smoothed_solution);
                new_makespan = shortcutter.calculate_makespan(smoothed_solution);
                new_flowtime = shortcutter.calculate_flowtime(smoothed_solution);
                log("Makespan reduced from " + std::to_string(makespan) + " to " + std::to_string(new_makespan), LogLevel::INFO);
            }
            else {
                smoothed_solution = rediscretized_solution;
            }
            removeWait(instance_, smoothed_solution);
            auto toc = std::chrono::high_resolution_clock::now();
            double t_plan = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() * 0.001;
            planning_time_ += t_plan;

            std::string pose_str = "task" + std::to_string(plans_.size());
            logProgressFileAppend(sc_options.progress_file, pose_str, pose_str, t_plan, flowtime, new_flowtime, makespan, new_makespan);

            plans_.push_back(smoothed_solution);
        }

        ROS_INFO("Planning time total %f s %d collision checks", getPlanningTime(), getPlanningColCheck());
        
        // create adg
        MRTrajectory sync_solution;
        concatSyncSolution(instance_, act_graph_, plans_, tpg_config.dt, sync_solution);

        auto adg = std::make_shared<tpg::ADG>(act_graph_);
        //adg->init_from_tpgs(planner.getInstance(), tpg_config, tpgs);
        adg->init_from_asynctrajs(instance_, tpg_config, sync_solution);
        set_tpg(adg);
        ROS_INFO("Adg init total %d collision checked ", getTPGColCheck() + instance_->numCollisionChecks());

        return true;
        
    }

    void checkFK(const std::vector<GoalPose> &poses) {
        kinematic_state_->setJointGroupPositions("left_arm", poses[0].joint_values);
        kinematic_state_->setJointGroupPositions("right_arm", poses[1].joint_values);
        kinematic_state_->update();
        
        const Eigen::Isometry3d& left_pose = kinematic_state_->getGlobalLinkTransform("left_arm_flange");
        const Eigen::Isometry3d& right_pose = kinematic_state_->getGlobalLinkTransform("right_arm_flange");
        Eigen::Quaterniond left_quat(left_pose.rotation());
        Eigen::Quaterniond right_quat(right_pose.rotation());
    
    }

    bool setLegoFactory(const std::string &config_fname, const std::string &root_pwd, const std::string &task_fname, const std::string &task_name)
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
        std::string env_setup_fname = root_pwd + config["env_setup_folder"].asString() + "env_setup_" + task_name + ".json";
        std::string lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();

        std::cout << "Task file: " << task_fname << std::endl;
        std::ifstream task_file(task_fname, std::ifstream::binary);
        task_file >> task_json_;
        num_tasks_ = task_json_.size();
        
        std::string world_base_fname = root_pwd + config["world_base_fname"].asString();

        bool assemble = config["Start_with_Assemble"].asBool();
        policy_cfg_.twist_rad = config["Twist_Deg"].asInt() * M_PI / 180.0;
        policy_cfg_.twist_rad_handover = config["Handover_Twist_Deg"].asInt() * M_PI / 180.0;

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

    bool initLegoPositions() {
        if (lego_ptr_ == nullptr) {
            ROS_ERROR("Lego pointer is not initialized");
            return false;
        }

        std::vector<std::string> brick_names = lego_ptr_->get_brick_names();
        addMoveitCollisionObject("table");
        instance_->setObjectColor("table", 0.0, 0.0, 1.0, 0.0);
        for (const auto & name : brick_names) {
            if (name.find("station") != std::string::npos) {
                continue;
            }
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
        
        ROS_INFO("Adding collision object %s at %f %f %f", name.c_str(), obj.x, obj.y, obj.z);
        ROS_INFO("Added collision object %s to world frame", name.c_str());
    }

    void attachMoveitCollisionObject(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose) {
        instance_->attachObjectToRobot(name, robot_id, link_name, pose);
        instance_->updateScene();

        ROS_INFO("Attached collision object %s to %s", name.c_str(), link_name.c_str());
    }

    void detachMoveitCollisionObject(const std::string &name, const RobotPose &pose) {
        instance_->detachObjectFromRobot(name, pose);
        instance_->updateScene();

        ROS_INFO("Detached collision object %s", name.c_str());
    }

    void getLegoBrickName(int task_idx, std::string &brick_name) {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asString());
    }

    int getRobot(int task_idx) {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        return cur_graph_node["robot_id"].asInt() - 1;
    }

    int getSupportRobot(int task_idx) {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        return cur_graph_node["sup_robot_id"].asInt() - 1;
    }

    int getManipType(int task_idx) {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        int manip_type = 0;
        if (cur_graph_node.isMember("manipulate_type")) {
            manip_type = cur_graph_node["manipulate_type"].asInt();
        }
        return manip_type;
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
        std::string brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asString());

        Eigen::Matrix4d brick_pose_mtx;
        lego_ptr_->calc_bric_asssemble_pose(brick_name, cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(),
                 cur_graph_node["z"].asInt(), cur_graph_node["ori"].asInt(), brick_pose_mtx);
        
        Object obj = instance_->getObject(brick_name);
        // define the object
        obj.state = Object::State::Static;
        obj.parent_link = "world";
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

    Object getLegoHandover(int task_idx, const RobotPose &start_pose) {
        auto cur_graph = task_json_[std::to_string(task_idx)];
        std::string brick_name = lego_ptr_->get_brick_name_by_id(cur_graph["brick_id"].asInt(), cur_graph["brick_seq"].asString());
        int sup_robot = cur_graph["sup_robot_id"].asInt() - 1;
        assert (sup_robot == start_pose.robot_id);
        int brick_id = cur_graph["brick_id"].asInt();
        int press_side = cur_graph["press_side"].asInt();
        int press_offset = cur_graph["press_offset"].asInt();


        Object obj = instance_->getObject(brick_name);
        // define the object
        obj.name = brick_name;
        obj.state = Object::State::Handover;
        obj.parent_link = eof_links_[sup_robot];
        obj.shape = Object::Shape::Box;

        // define the object
        Eigen::Matrix4d brick_loc;
        lego_manipulation::math::VectorJd press_joints = Eigen::MatrixXd::Zero(6, 1);
        press_joints << start_pose.joint_values[0], start_pose.joint_values[1], start_pose.joint_values[2], 
                        start_pose.joint_values[3], start_pose.joint_values[4], start_pose.joint_values[5];
        press_joints = press_joints * M_PI / 180.0;
        lego_ptr_->lego_pose_from_press_pose(press_joints, sup_robot, brick_id, 
            press_side, press_offset, brick_loc);
        
        obj.x = brick_loc(0, 3);
        obj.y = brick_loc(1, 3);
        obj.z = brick_loc(2, 3) - obj.height/2;
        Eigen::Quaterniond quat(brick_loc.block<3, 3>(0, 0));
        obj.qx = quat.x();
        obj.qy = quat.y();
        obj.qz = quat.z();
        obj.qw = quat.w();
         
        lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);
        return obj;
    }

    bool setCollision(const std::string& object_id, const std::string& link_name, bool allow) {
        instance_->setCollision(object_id, link_name, allow);
        instance_->updateScene();

        if (allow)
            ROS_INFO("Allow collision between %s and %s", object_id.c_str(), link_name.c_str());
        else
            ROS_INFO("Disallow collision between %s and %s", object_id.c_str(), link_name.c_str());

        return true;
    }

    void left_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 6; i++) {
            current_joints_[i] = msg->position[i];
        }
        left_arm_joint_state_received = true;

        if (tpg_ != nullptr) {
            tpg_->update_joint_states(msg->position, 0);
        }

    }

    void right_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 6; i++) {
            current_joints_[7+i] = msg->position[i];
        }
        right_arm_joint_state_received = true;

        if (tpg_ != nullptr) {
            tpg_->update_joint_states(msg->position, 1);
        }
        
    }

    void dual_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        if (msg->position.size() != 14) {
            ROS_ERROR("Received joint state message with size %lu", msg->position.size());
            return;
        }
        for (size_t i = 0; i < 7; i++) {
            current_joints_[i] = msg->position[i];
            current_joints_[7+i] = msg->position[7+i];
        }
        left_arm_joint_state_received = true;
        right_arm_joint_state_received = true;

        std::vector<double> left_joints(msg->position.begin(), msg->position.begin()+7);
        std::vector<double> right_joints(msg->position.begin()+7, msg->position.begin()+14);

        r1_j1_msg.data = left_joints[0];
        r1_j2_msg.data = left_joints[1];
        r1_j3_msg.data = left_joints[2];
        r1_j4_msg.data = left_joints[3];
        r1_j5_msg.data = left_joints[4];
        r1_j6_msg.data = left_joints[5];

        r2_j1_msg.data = right_joints[0];
        r2_j2_msg.data = right_joints[1];
        r2_j3_msg.data = right_joints[2];
        r2_j4_msg.data = right_joints[3];
        r2_j5_msg.data = right_joints[4];
        r2_j6_msg.data = right_joints[5];

        r1_j1_pub.publish(r1_j1_msg);
        r1_j2_pub.publish(r1_j2_msg);
        r1_j3_pub.publish(r1_j3_msg);
        r1_j4_pub.publish(r1_j4_msg);
        r1_j5_pub.publish(r1_j5_msg);
        r1_j6_pub.publish(r1_j6_msg);

        r2_j1_pub.publish(r2_j1_msg);
        r2_j2_pub.publish(r2_j2_msg);
        r2_j3_pub.publish(r2_j3_msg);
        r2_j4_pub.publish(r2_j4_msg);
        r2_j5_pub.publish(r2_j5_msg);
        r2_j6_pub.publish(r2_j6_msg);

        if (tpg_ != nullptr) {
            // get in-hands objects before current state update
            std::string pre_r1_inhand_obj_name = tpg_->get_r1_inhand_obj_name();
            std::string pre_r2_inhand_obj_name = tpg_->get_r2_inhand_obj_name();

            tpg_->update_joint_states(left_joints, 0);
            tpg_->update_joint_states(right_joints, 1);

            // get in-hand objects after current state update
            std::string r1_inhand_obj_name = tpg_->get_r1_inhand_obj_name();
            std::string r2_inhand_obj_name = tpg_->get_r2_inhand_obj_name();
            
            int r1_mode = 0, r2_mode = 0;
            std::shared_ptr<ActivityGraph> act_graph = tpg_->getActGraph();
            int r1_act = tpg_->getExecutedAct(0);
            int r2_act = tpg_->getExecutedAct(1);
            lego_manipulation::math::VectorJd robot1_start = Eigen::MatrixXd::Zero(6, 1);
            lego_manipulation::math::VectorJd robot2_start = Eigen::MatrixXd::Zero(6, 1);
            if (act_graph != nullptr) {
                auto r1_act_ptr = act_graph->get(0, r1_act);
                auto r2_act_ptr = act_graph->get(1, r2_act);
                if (r1_act_ptr != nullptr) {
                    r1_mode = r1_act_ptr->type; 
                    robot1_start << r1_act_ptr->start_pose.joint_values[0], 
                                    r1_act_ptr->start_pose.joint_values[1], 
                                    r1_act_ptr->start_pose.joint_values[2], 
                                    r1_act_ptr->start_pose.joint_values[3], 
                                    r1_act_ptr->start_pose.joint_values[4], 
                                    r1_act_ptr->start_pose.joint_values[5];
                }
                if (r2_act_ptr != nullptr) {
                    r2_mode = r2_act_ptr->type;
                    robot2_start << r2_act_ptr->start_pose.joint_values[0], 
                                    r2_act_ptr->start_pose.joint_values[1], 
                                    r2_act_ptr->start_pose.joint_values[2], 
                                    r2_act_ptr->start_pose.joint_values[3], 
                                    r2_act_ptr->start_pose.joint_values[4], 
                                    r2_act_ptr->start_pose.joint_values[5];
                }
            }

            lego_manipulation::math::VectorJd robot1_q = Eigen::MatrixXd::Zero(6, 1);
            lego_manipulation::math::VectorJd robot2_q = Eigen::MatrixXd::Zero(6, 1);
            robot1_q << left_joints[0], left_joints[1], left_joints[2], left_joints[3], left_joints[4], left_joints[5];
            robot2_q << right_joints[0], right_joints[1], right_joints[2], right_joints[3], right_joints[4], right_joints[5];
            
            // send lego brick state to the gazebo simulator
            if (lego_ptr_ != nullptr) {
                if(r1_inhand_obj_name.compare("None") == 0 && pre_r1_inhand_obj_name.compare("None") != 0)
                {
                    if(r1_mode < 25)
                    {
                        lego_ptr_->update_bricks(robot1_start, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 1, pre_r1_inhand_obj_name, 0);
                    }
                    else
                    {
                        lego_ptr_->update_bricks(robot1_start, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), 1, pre_r1_inhand_obj_name, 1);
                    }
                }
                else if(r1_inhand_obj_name.compare("None") != 0)
                {
                    // objects are in hand
                    if((r1_mode >= 4 && r1_mode <= 9) || (r1_mode >= 21 && r1_mode <= 23))
                    {
                        lego_ptr_->update_bricks(robot1_q, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 1, r1_inhand_obj_name, 0);
                    }
                    else if((r1_mode >= 19 && r1_mode <= 20) || (r1_mode >= 26 && r1_mode <= 29))
                    {
                        lego_ptr_->update_bricks(robot1_q, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), 1, r1_inhand_obj_name, 1);
                    }
                }
                
                if(r2_inhand_obj_name.compare("None") == 0 && pre_r2_inhand_obj_name.compare("None") != 0)
                {
                    if(r2_mode < 25)
                    {
                        lego_ptr_->update_bricks(robot2_start, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 1, pre_r2_inhand_obj_name, 0);
                    }
                    else
                    {
                        lego_ptr_->update_bricks(robot2_start, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), 1, pre_r2_inhand_obj_name, 1);
                    }
                }
                else if(r2_inhand_obj_name.compare("None") != 0)
                {
                    if((r2_mode >= 4 && r2_mode <= 9) || (r2_mode >= 21 && r2_mode <= 23))
                    {
                        lego_ptr_->update_bricks(robot2_q, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 1, r2_inhand_obj_name, 0);
                    }
                    else if((r2_mode >= 19 && r2_mode <= 20) || (r2_mode >= 26 && r2_mode <= 29))
                    {
                        lego_ptr_->update_bricks(robot2_q, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), 1, r2_inhand_obj_name, 1);
                    }
                }
            }
        }
    }

    void get_brick_corners(const Object &obj, double &lx, double &ly, double &rx, double &ry) {
        double yaw = atan2(2.0*(obj.qx*obj.qy + obj.qw*obj.qz), obj.qw*obj.qw + obj.qx*obj.qx - obj.qy*obj.qy - obj.qz*obj.qz);
        yaw = yaw * 180.0 / M_PI;
        double eps = 5;
        if ((std::abs(yaw - 90) < eps) || (std::abs(yaw + 90) < eps)) {
            lx = obj.x - obj.width/2;
            rx = obj.x + obj.width/2;
            ly = obj.y - obj.length/2;
            ry = obj.y + obj.length/2;   
        } else {
            lx = obj.x - obj.length/2;
            rx = obj.x + obj.length/2;
            ly = obj.y - obj.width/2;
            ry = obj.y + obj.width/2;
        }
    }

    bool brick_overlap(const Object &obj1, const Object &obj2) {
        double lx1, ly1, rx1, ry1, lx2, ly2, rx2, ry2;
        get_brick_corners(obj1, lx1, ly1, rx1, ry1);
        get_brick_corners(obj2, lx2, ly2, rx2, ry2);

        if (lx1 > rx2 || lx2 > rx1) {
            return false;
        }
        if (ly1 > ry2 || ly2 > ry1) {
            return false;
        }

        return true;
    }
    
    void getLegoObjects(const std::string &brick_name, int task_idx, bool target_pose, std::vector<std::string> &objects, bool is_top) {
        // loop over all blocks, check if it is in previous task.
        // if yes, use target position, otherwise use initial position
        objects.clear();

        std::vector<std::string> brick_names = lego_ptr_->get_brick_names();
        Object cur_obj;
        if (target_pose) {
            cur_obj = getLegoTarget(task_idx);
        } else {
            cur_obj = getLegoStart(brick_name);
        }

        std::map<std::string, int> moved_bricks;
        for (int tid = 1; tid < task_idx; tid++) {
            std::string brick_name;
            getLegoBrickName(tid, brick_name);
            moved_bricks[brick_name] = tid;
        }

        for (const auto & name : brick_names) {
            if (name == brick_name) {
                continue;
            }
            Object obj;
            auto it = moved_bricks.find(name);
            if (it != moved_bricks.end()) {
                obj = getLegoTarget(it->second);
            } else {
                obj = getLegoStart(name);
            }

            double z_diff = is_top ? (obj.z - obj.height/2 - cur_obj.z - cur_obj.height/2 )
                                : (cur_obj.z - cur_obj.height/2 - obj.z - obj.height/2);
            if (std::abs(z_diff) > 0.005) {
                continue;
            }
            if (brick_overlap(cur_obj, obj)) {
                objects.push_back(name);
            }
        }

        if (objects.size() == 0 && !is_top) {
            objects.push_back("table");
        }

        for (const auto & obj : objects) {
            ROS_INFO("%s object: %s", is_top ? "Top" : "Bottom", obj.c_str());
        }
    }

    void getLegoBottom(const std::string &brick_name, int task_idx, bool target_pose, std::vector<std::string> &bot_objects) {
        getLegoObjects(brick_name, task_idx, target_pose, bot_objects, false);
    }

    void getLegoTop(const std::string &brick_name, int task_idx, bool target_pose, std::vector<std::string> &top_objects) {
        getLegoObjects(brick_name, task_idx, target_pose, top_objects, true);
    }

    void getLegoTwistNext(int task_idx, const std::string &brick_name, std::vector<std::string> &side_bricks) {
        lego_ptr_->get_lego_twist_next(task_json_, task_idx, brick_name, side_bricks);
    }

    void getLegoSuppNearby(int task_idx, std::vector<std::string> &side_bricks) {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        if (!cur_graph_node.isMember("support_x")) {
            log("No support positions for task " + std::to_string(task_idx) + " in task json", LogLevel::WARN);
            return;
        }
        int support_x = cur_graph_node["support_x"].asInt();
        int support_y = cur_graph_node["support_y"].asInt();
        int support_z = cur_graph_node["support_z"].asInt();
        int support_ori = cur_graph_node["support_ori"].asInt();
        if (support_x == -1) {
            log("Support not necessary for task " + std::to_string(task_idx), LogLevel::WARN);
            return;
        }
        int sup_press_side, sup_brick_ori;
        lego_ptr_->get_sup_side_ori(support_ori, sup_press_side, sup_brick_ori);
        
        std::vector<std::vector<std::vector<std::string>>> world_grid = lego_ptr_->gen_world_grid_from_graph(task_json_, task_idx, 48, 48, 48);

        std::vector<std::string> above_bricks, side_low_bricks;
        lego_ptr_->get_lego_next(support_x, support_y, support_z, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid, side_bricks);
        lego_ptr_->get_lego_next(support_x, support_y, support_z-1, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid, side_low_bricks);
        lego_ptr_->get_lego_above(support_x, support_y, support_z, sup_brick_ori, 9, world_grid, above_bricks);
        
        // add above bricks to side bricks
        side_bricks.insert(side_bricks.end(), above_bricks.begin(), above_bricks.end());
        side_bricks.insert(side_bricks.end(), side_low_bricks.begin(), side_low_bricks.end());
    }

    std::shared_ptr<PlanInstance> getInstance() {
        return instance_;
    }

    std::shared_ptr<tpg::TPG> copyCurrentTPG() {
        return std::make_shared<tpg::TPG>(*tpg_);
    }

    std::vector<double> getCurrentJoints() {
        return current_joints_;
    }

    double getPlanningTime() {
        return planning_time_;
    }

    int getPlanningColCheck() {
        return planning_colcheck_;
    }

    int getTPGColCheck() {
        return tpg_colcheck_;
    }

    void setForceThreshold(double force_threshold, double placeup_force_threshold, double handover_force_threshold,
                        double drop_thresh_w_sup) {
        policy_cfg_.z_force_threshold = force_threshold;
        policy_cfg_.z_force_thresh_w_sup = drop_thresh_w_sup;
        policy_cfg_.x_force_threshold = placeup_force_threshold;
        policy_cfg_.handover_force_threshold = handover_force_threshold;
    }

    void setSupForceTolerance(double force_tol) {
        policy_cfg_.sup_force_tol = force_tol;
    }

    void setPlanningTimeLimit(double time_limit) {
        planning_time_limit_ = time_limit;
    }

    void setupGazeboPub(ros::NodeHandle nh)
    {
        r1_j1_topic = "/r1/joint1_position_controller/command";
        r1_j2_topic = "/r1/joint2_position_controller/command";
        r1_j3_topic = "/r1/joint3_position_controller/command";
        r1_j4_topic = "/r1/joint4_position_controller/command";
        r1_j5_topic = "/r1/joint5_position_controller/command";
        r1_j6_topic = "/r1/joint6_position_controller/command";
        r1_j1_pub = nh.advertise<std_msgs::Float64>(r1_j1_topic, 1);
        r1_j2_pub = nh.advertise<std_msgs::Float64>(r1_j2_topic, 1);
        r1_j3_pub = nh.advertise<std_msgs::Float64>(r1_j3_topic, 1);
        r1_j4_pub = nh.advertise<std_msgs::Float64>(r1_j4_topic, 1);
        r1_j5_pub = nh.advertise<std_msgs::Float64>(r1_j5_topic, 1);
        r1_j6_pub = nh.advertise<std_msgs::Float64>(r1_j6_topic, 1);

        r2_j1_topic = "/r2/joint1_position_controller/command";
        r2_j2_topic = "/r2/joint2_position_controller/command";
        r2_j3_topic = "/r2/joint3_position_controller/command";
        r2_j4_topic = "/r2/joint4_position_controller/command";
        r2_j5_topic = "/r2/joint5_position_controller/command";
        r2_j6_topic = "/r2/joint6_position_controller/command";
        r2_j1_pub = nh.advertise<std_msgs::Float64>(r2_j1_topic, 1);
        r2_j2_pub = nh.advertise<std_msgs::Float64>(r2_j2_topic, 1);
        r2_j3_pub = nh.advertise<std_msgs::Float64>(r2_j3_topic, 1);
        r2_j4_pub = nh.advertise<std_msgs::Float64>(r2_j4_topic, 1);
        r2_j5_pub = nh.advertise<std_msgs::Float64>(r2_j5_topic, 1);
        r2_j6_pub = nh.advertise<std_msgs::Float64>(r2_j6_topic, 1);
    }

    std::shared_ptr<tpg::ADG> getAdg() {
        return std::dynamic_pointer_cast<tpg::ADG>(tpg_);
    }


private:
    ros::NodeHandle nh_;
    
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr kinematic_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene::PlanningScenePtr planning_scene_;
    std::vector<std::string> eof_links_ = {"left_arm_link_tool", "right_arm_link_tool"};
    std::vector<std::vector<std::string>> robot_links_;

    // lego pointer
    lego_manipulation::lego::Lego::Ptr lego_ptr_;
    Json::Value task_json_;
    int num_tasks_ = 0;

    // update gazebo state
    ros::ServiceClient set_state_client_;
    ros::ServiceClient planning_scene_diff_client;

    // joint names for execution
    std::vector<std::string> joint_names_;
    std::vector<std::vector<std::string>> joint_names_split_;
    std::vector<double> current_joints_;
    bool left_arm_joint_state_received = false;
    bool right_arm_joint_state_received = false;
    ros::Subscriber left_arm_sub, right_arm_sub, dual_arm_sub;

    // tpg execution
    std::shared_ptr<tpg::TPG> tpg_;
    std::shared_ptr<ActivityGraph> act_graph_;

    bool async_ = false;
    bool mfi_ = false;
    bool fake_move_ = false;
    std::vector<std::string> group_names_; // group name for moveit controller services

    std::string planner_type_;
    std::shared_ptr<MoveitInstance> instance_;

    std::vector<MRTrajectory> plans_;

    std::string output_dir_;
    int counter_ = 0;
    double planning_time_ = 0.0;
    double planning_time_limit_ = 5.0;
    int planning_colcheck_ = 0;
    int tpg_colcheck_ = 0;
    LegoPolicyCfg policy_cfg_;

    // Gazebo
    std::string r1_j1_topic, r1_j2_topic, r1_j3_topic, r1_j4_topic, r1_j5_topic, r1_j6_topic;
    std::string r2_j1_topic, r2_j2_topic, r2_j3_topic, r2_j4_topic, r2_j5_topic, r2_j6_topic;
    ros::Publisher r1_j1_pub, r1_j2_pub, r1_j3_pub, r1_j4_pub, r1_j5_pub, r1_j6_pub;
    ros::Publisher r2_j1_pub, r2_j2_pub, r2_j3_pub, r2_j4_pub, r2_j5_pub, r2_j6_pub;
    std_msgs::Float64 r1_j1_msg, r1_j2_msg, r1_j3_msg, r1_j4_msg, r1_j5_msg, r1_j6_msg;
    std_msgs::Float64 r2_j1_msg, r2_j2_msg, r2_j3_msg, r2_j4_msg, r2_j5_msg, r2_j6_msg;

};



int main(int argc, char** argv) {
    // Set stack size to 65536 KB
    rlimit rlim;
    rlim.rlim_cur = 65536 * 1024; // current soft limit
    rlim.rlim_max = 65536 * 1024; // maximum hard limit
    if (setrlimit(RLIMIT_STACK, &rlim) != 0) {
        // Handle error
        perror("setrlimit failed");
    }

    ros::init(argc, argv, "lego_node");
    
    // start ros node
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Read ROS Params
    std::string file_path, planner_type, config_fname, root_pwd, output_dir, task_fname, task_name;
    bool async = false;
    bool mfi = false;
    bool load_tpg = false;
    bool load_adg = false;
    bool benchmark = false;
    bool fake_move = false;
    bool print_debug = false;
    bool sync_plan = false;
    double vmax = 1.0;
    double planning_time_limit = 5.0;
    int start_task_idx = 1;
    std::vector<std::string> group_names = {"left_arm", "right_arm"};
    for (int i = 0; i < 2; i++) {
        if (nh.hasParam("group_name_" + std::to_string(i))) {
            nh.getParam("group_name_" + std::to_string(i), group_names[i]);
       }
    }

    nh.getParam("fullorder_targets_filename", file_path);
    nh.getParam("planner_type", planner_type);
    nh.param<double>("planning_time_limit", planning_time_limit, 5.0);
    nh.param<std::string>("config_fname", config_fname, "");
    nh.param<std::string>("task_fname", task_fname, "");
    nh.param<std::string>("task_name", task_name, "");
    nh.param<std::string>("root_pwd", root_pwd, "");
    nh.param<std::string>("output_dir", output_dir, "");
    nh.param<bool>("async", async, false);
    nh.param<bool>("mfi", mfi, false);
    nh.param<bool>("load_tpg", load_tpg, false);
    nh.param<bool>("load_adg", load_adg, false);
    nh.param<bool>("benchmark", benchmark, false);
    nh.param<bool>("fake_move", fake_move, false);
    nh.param<int>("start_task_idx", start_task_idx, 1);
    nh.param<bool>("print_debug", print_debug, false);
    if (mfi || load_tpg || load_adg) {
        fake_move = false;
    }

    tpg::TPGConfig tpg_config;

    // Initialize the Dual Arm Planner
    if (print_debug) {
        setLogLevel(LogLevel::DEBUG);
        tpg_config.print_contact = true;
    } else {
        setLogLevel(LogLevel::INFO);
    }

    nh.param<double>("adg_shortcut_time", tpg_config.shortcut_time, 0.1);
    nh.param<bool>("random_shortcut", tpg_config.random_shortcut, true);
    nh.param<bool>("tight_shortcut", tpg_config.tight_shortcut, true);
    nh.param<bool>("subset_shortcut", tpg_config.subset_shortcut, false);
    nh.param<double>("subset_prob", tpg_config.subset_prob, 0.4);
    nh.param<bool>("forward_doubleloop", tpg_config.forward_doubleloop, false);
    nh.param<bool>("backward_doubleloop", tpg_config.backward_doubleloop, false);
    nh.param<bool>("forward_singleloop", tpg_config.forward_singleloop, false);
    nh.param<bool>("biased_sample", tpg_config.biased_sample, false);
    nh.param<int>("seed", tpg_config.seed, 1);
    nh.param<bool>("run_policy", tpg_config.run_policy, false);
    nh.param<std::string>("progress_file", tpg_config.progress_file, "");
    nh.param<double>("vmax", vmax, 1.0);
    nh.param<double>("joint_state_thresh", tpg_config.joint_state_thresh, 0.1);
    nh.param<bool>("parallel", tpg_config.parallel, false);
    nh.param<bool>("sync_plan", sync_plan, false);
    nh.param<bool>("sync_exec", tpg_config.sync_task, false);
    tpg_config.dt = 0.05 / vmax;
    ROS_INFO("TPG Config: vmax: %f, dt: %f, shortcut_time: %f, tight_shortcut: %d", vmax, tpg_config.dt, tpg_config.shortcut_time, tpg_config.tight_shortcut);
    
    ShortcutOptions sc_options;
    nh.param<double>("sync_shortcut_time", sc_options.t_limit, 2.0);
    sc_options.prioritized_shortcut = true;
    sc_options.dt = tpg_config.dt;
    sc_options.tight_shortcut = tpg_config.tight_shortcut;
    sc_options.log_interval = tpg_config.log_interval;
    sc_options.progress_file = tpg_config.progress_file;
    sc_options.seed = tpg_config.seed;


    double force_threshold, placeup_force_threshold, sup_force_tol, drop_thresh_w_sup, handover_force_threshold;
    nh.param<double>("force_threshold", force_threshold, -18.0);
    nh.param<double>("drop_thresh_w_sup", drop_thresh_w_sup, -5.0);
    nh.param<double>("placeup_force_threshold", placeup_force_threshold, 6.0);
    nh.param<double>("sup_force_tol", sup_force_tol, 0.1);
    nh.param<double>("handover_force_threshold", handover_force_threshold, -10.0);
    DualArmPlanner planner(planner_type, output_dir, group_names, async, mfi, fake_move, vmax);
    planner.setForceThreshold(force_threshold, placeup_force_threshold, handover_force_threshold, drop_thresh_w_sup);
    planner.setSupForceTolerance(sup_force_tol);
    planner.setPlanningTimeLimit(planning_time_limit);
    planner.setupGazeboPub(nh);

    // wait 2 seconds
    ros::Duration(2).sleep();
    planner.setup_once();
    ROS_INFO("Execution setup done");

    // Read the robot poses
    std::vector<std::vector<GoalPose>> all_poses;
    readPosesFromFile(file_path, all_poses);
    ROS_INFO("Read %lu poses", all_poses.size());

    // Read the lego poses
    if (!config_fname.empty() && !root_pwd.empty()) {
        planner.setLegoFactory(config_fname, root_pwd, task_fname, task_name);
        planner.initLegoPositions();
    }

    // clear log file
    logProgressFileStart(tpg_config.progress_file);

    // Start Execution Loop
    if (load_adg) {
        // open the file safely
        std::string filename = output_dir + "/adg.txt";
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return -1;
        }
        auto adg = std::make_shared<tpg::ADG>();
        boost::archive::text_iarchive ia(ifs);
        ia >> adg;
    
        bool success = adg->optimize(planner.getInstance(), tpg_config);
        if (!success) {
            ROS_ERROR("Failed to optimize ADG");
            return -1;
        }
        if (!benchmark) {
            planner.getInstance()->resetScene(true);
            planner.initLegoPositions();
            planner.set_tpg(adg);
            for (int r = 0; r < 2; r++) {
                int start_act_id = (start_task_idx-1) * 13;
                adg->setExecStartAct(r, start_act_id);
            }
            planner.reset_joint_states_flag();
            planner.execute(adg);
        }
        adg->saveStats(tpg_config.progress_file);

        ros::shutdown();
        return 0;
    }

    bool success;
    if (sync_plan) {
        success = planner.sync_plan(start_task_idx, load_tpg, all_poses,
                        sc_options, tpg_config);
    }
    else {
        success = planner.sequential_plan(start_task_idx, load_tpg, all_poses,
                        sc_options, tpg_config, false);
    }
    if (!success)
        return -1;
    
    std::shared_ptr<tpg::ADG> adg = planner.getAdg();
    auto act_graph = adg->getActGraph();

    // save act graph
    act_graph->saveGraphToFile(output_dir + "/activity_graph.dot");

    // optimize adg
    success = adg->optimize(planner.getInstance(), tpg_config);
    if (!success) {
        ROS_ERROR("Failed to optimize ADG");
        return -1;
    }
    adg->saveStats(tpg_config.progress_file);
    
    // save adg
    if (!benchmark) {
        adg->saveToDotFile(output_dir + "/adg.dot");
        
        // get the synchronized plan
        MRTrajectory traj = adg->getSyncJointTrajectory(planner.getInstance());
        saveSolution(planner.getInstance(), traj, output_dir + "/full_solution.csv");
    
        std::ofstream ofs(output_dir + "/adg.txt");
        if (!ofs.is_open()) {
            ROS_ERROR("Failed to open file: %s", (output_dir + "/adg.txt").c_str());
            return -1;
        }
        boost::archive::text_oarchive oa(ofs);
        oa << adg;
        ofs.close();
        ROS_INFO("Saved ADG to file");

        // execute adg
        planner.setup_once();
        planner.getInstance()->resetScene(true);
        planner.initLegoPositions();
        planner.set_tpg(adg);
        planner.reset_joint_states_flag();
        planner.execute(adg);
    }

    ROS_INFO("Planning completed successfully");
    ros::shutdown();
    return 0;
}
