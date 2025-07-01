/**
 * @file instance.h
 * @brief Abstract planning interface and collision environment management
 * 
 * This file provides a unified interface for motion planning across different robot
 * configurations and planning frameworks. It manages dynamic collision environments
 * during multi-robot assembly planning and provides abstractions for different
 * robot hardware interfaces.
 * 
 * Key responsibilities:
 * - Robot model management and configuration
 * - Dynamic collision environment updates during assembly
 * - Planning scene synchronization across multiple robots
 * - Hardware abstraction for different robot types
 * - Object attachment and detachment tracking
 * 
 * @author Philip Huang
 * @date 2025
 */

#ifndef APEX_MR_INSTANCE_H
#define APEX_MR_INSTANCE_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
//#include <moveit/collision_detection_bullet/collision_env_bullet.h>
//#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <std_msgs/ColorRGBA.h>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <memory>
#include <vector>
#include <random>
#include <Eigen/Geometry>
#include <chrono>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <set>

// Abstract base class for the planning scene interface

struct Object  {
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & name;
        ar & state;
        ar & parent_link;
        ar & robot_id;
        ar & x;
        ar & y;
        ar & z;
        ar & qx;
        ar & qy;
        ar & qz;
        ar & qw;
        ar & shape;
        ar & radius;
        ar & length;
        ar & width;
        ar & height;
        ar & mesh_path;
    }    
    
    enum State {
        Static = 0,
        Attached = 1,
        Supported = 2,
        Handover = 3,
    };
    enum Shape {
        Box = 0,
        Sphere = 1,
        Cylinder = 2,
        Mesh = 3,
    };

    Object() = default;
    Object(const std::string &name, const std::string& parent_link, State state, double x, double y, double z, double qx, double qy, double qz, double qw):
        name(name), parent_link(parent_link), state(state), x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw) 
        {}
    
    std::string name;
    // mode of the object
    State state;
    std::string parent_link;
    int robot_id = -1;

    // geometry of the object
    double x, y, z;
    double qx = 0, qy = 0, qz = 0, qw = 1.0;
    double x_attach, y_attach, z_attach;
    double qx_attach = 0, qy_attach = 0, qz_attach = 0, qw_attach = 0;

    // collision shape of the object
    Shape shape;
    double radius;
    double length; // x
    double width; // y
    double height; // z
    std::string mesh_path;
};

struct RobotMode {

    enum Type {
        Free = 0,
        Carry = 1,
        Hold = 2,
    };
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & type;
        ar & carried_obj;
        ar & held_obj;
        ar & ee_link;
        ar & obj;
    }
    
    Type type = Free;
    std::string carried_obj;
    std::string held_obj;
    std::string ee_link;
    std::shared_ptr<Object> obj;
};

struct RobotPose {
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & robot_id;
        ar & robot_name;
        ar & joint_values;
        ar & hand_values;
    }
    int robot_id;
    // RobotMode mode;
    std::string robot_name; // same as group name in moveit
    std::vector<double> joint_values;
    std::vector<double> hand_values;

    // Define the equality operator
    bool operator==(const RobotPose& other) const {
        return robot_id == other.robot_id &&
               robot_name == other.robot_name &&
               joint_values == other.joint_values &&
               hand_values == other.hand_values;
    }

};

struct RobotTrajectory {
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & robot_id;
        ar & trajectory;
        ar & times;
        ar & cost;
    }
    int robot_id;
    std::vector<RobotPose> trajectory;
    std::vector<double> times;
    std::vector<int> act_ids;
    double cost;
};

typedef std::vector<RobotTrajectory> MRTrajectory;

// Forward declaration of the hash function
namespace std {
    template <>
    struct hash<RobotPose> {
        std::size_t operator()(const RobotPose& pose) const {
            std::size_t h1 = std::hash<int>()(pose.robot_id);
            std::size_t h2 = std::hash<std::string>()(pose.robot_name);

            // Custom hash function for std::vector<double>
            auto hash_vector = [](const std::vector<double>& vec) {
                std::size_t seed = 0;
                for (double val : vec) {
                    seed ^= std::hash<double>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                }
                return seed;
            };

            std::size_t h3 = hash_vector(pose.joint_values);
            std::size_t h4 = hash_vector(pose.hand_values);

            return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3); // Combine the hash values
        }
    };
}

class PlanInstance {
public:
    virtual void setNumberOfRobots(int num_robots);
    virtual void setRobotNames(const std::vector<std::string>& robot_names) {
        robot_names_ = robot_names;
    }
    virtual void setHandNames(const std::vector<std::string>& hand_names) {
        hand_names_ = hand_names;
    }
    virtual void setStartPose(int robot_id, const std::vector<double>& pose);
    virtual void setGoalPose(int robot_id, const std::vector<double>& pose);
    virtual bool checkCollision(const std::vector<RobotPose> &poses, bool self, bool debug=false) = 0;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b) const = 0;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b, int dim) const = 0;
    virtual bool connect(const RobotPose& a, const RobotPose& b, double col_step_size = 0.1, bool debuf=false) = 0;
    virtual bool steer(const RobotPose& a, const RobotPose& b, double max_dist,  RobotPose& result, double col_step_size = 0.1) = 0;
    virtual bool sample(RobotPose &pose) = 0;
    virtual double getVMax(int robot_id);
    virtual void setVmax(double vmax);
    virtual RobotPose interpolate(const RobotPose &a, const RobotPose&b, double t) const = 0;
    virtual double interpolate(const RobotPose &a, const RobotPose&b, double t, int dim) const = 0;
    virtual shape_msgs::SolidPrimitive getPrimitive(const Object& obj);
    virtual geometry_msgs::Pose getPose(const Object& obj);
    virtual void addMoveableObject(const Object& obj) { throw std::runtime_error("Not implemented");};
    virtual void moveObject(const Object& obj) { throw std::runtime_error("Not implemented");};
    virtual void removeObject(const std::string& name) { throw std::runtime_error("Not implemented");};
    virtual void moveRobot(int robot_id, const RobotPose& pose) { throw std::runtime_error("Not implemented");};
    virtual void attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose) { throw std::runtime_error("Not implemented");};
    virtual void detachObjectFromRobot(const std::string& name, const RobotPose &pose) { throw std::runtime_error("Not implemented");};
    virtual void updateScene() = 0;
    virtual void resetScene(bool reset_sim) = 0;
    virtual void setPadding(double padding) {throw std::runtime_error("Not implemented");};
    virtual bool setCollision(const std::string& obj_name, const std::string& link_name, bool allow) { throw std::runtime_error("Not implemented");};
    virtual void printKnownObjects() const { throw std::runtime_error("Not implemented");};
    virtual int numCollisionChecks();
    // Additional methods for future functionalities can be added here
    virtual ~PlanInstance() = default;

    virtual int getNumberOfRobots() const {
        return num_robots_;
    }

    virtual std::vector<RobotPose> getStartPoses() const {
        return start_poses_;
    }

    virtual std::vector<RobotPose> getGoalPoses() const {
        return goal_poses_;
    }

    virtual RobotPose getStartPose(int robot_id) const {
        assert (robot_id < start_poses_.size());
        return start_poses_[robot_id];
    }

    virtual RobotPose getGoalPose(int robot_id) const {
        assert (robot_id < goal_poses_.size());
        return goal_poses_[robot_id];
    }

    virtual RobotPose initRobotPose(int robot_id) const;

    virtual void setRobotDOF(int robot_id, size_t dof);

    virtual void setHandDof(int robot_id, size_t dof);

    virtual size_t getRobotDOF(int robot_id) const;

    virtual size_t getHandDOF(int robot_id) const;

    virtual bool hasObject(const std::string& name) const {
        return objects_.find(name) != objects_.end();
    }

    virtual Object getObject(const std::string& name) const {
        return objects_.at(name);
    }

    virtual std::vector<Object> getAttachedObjects(int robot_id) const {
        std::vector<Object> attached_objects;
        for (const auto& obj : objects_) {
            if (obj.second.robot_id == robot_id && obj.second.state == Object::State::Attached) {
                attached_objects.push_back(obj.second);
            }
        }
        return attached_objects;
    }

protected:
    int num_robots_;
    double v_max_ = 1.0;
    std::vector<RobotPose> start_poses_;
    std::vector<size_t> robot_dof_, hand_dof_;
    std::vector<RobotPose> goal_poses_;
    std::vector<std::string> robot_names_, hand_names_;
    std::unordered_map<std::string, Object> objects_;

    int num_collision_checks_ = 0;
};

// Concrete implementation using MoveIt
class MoveitInstance : public PlanInstance {
public:
    MoveitInstance(robot_state::RobotStatePtr kinematic_state,
                   const std::string &joint_group_name,
                   planning_scene::PlanningScenePtr planning_scene);
    virtual bool checkCollision(const std::vector<RobotPose> &poses, bool self, bool debug=false) override;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b) const override;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b, int dof) const override;
    virtual bool connect(const RobotPose& a, const RobotPose& b, double col_step_size = 0.1, bool debug=false) override;
    virtual bool steer(const RobotPose& a, const RobotPose& b, double max_dist, RobotPose& result, double col_step_size = 0.1) override;
    virtual bool sample(RobotPose &pose) override;
    virtual RobotPose interpolate(const RobotPose &a, const RobotPose&b, double t) const override;
    virtual double interpolate(const RobotPose &a, const RobotPose&b, double t, int dof) const override;
    // Implementation of abstract methods using MoveIt functionalities
    virtual void addMoveableObject(const Object& obj) override;
    virtual void moveObject(const Object& obj) override;
    virtual void removeObject(const std::string& name) override;
    virtual void moveRobot(int robot_id, const RobotPose& pose) override;
    virtual void attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose) override;
    virtual void detachObjectFromRobot(const std::string& name, const RobotPose &pose) override;
    virtual void setObjectColor(const std::string &name, double r, double g, double b, double a);
    virtual moveit_msgs::PlanningScene getPlanningSceneDiff() const {
        return planning_scene_diff_;
    }
    virtual void setPlanningSceneDiffClient(ros::ServiceClient &client) {
        planning_scene_diff_client_ = client;
    }
    virtual void updateScene() override;
    virtual void resetScene(bool reset_sim) override;
    virtual void setPadding(double padding) override;

    virtual bool setCollision(const std::string& obj_name, const std::string& link_name, bool allow) override;
    virtual void printKnownObjects() const override;

private:
    // moveit move_group and planning_scene_interface pointers
    std::string joint_group_name_;
    robot_state::RobotStatePtr kinematic_state_;
    planning_scene::PlanningScenePtr planning_scene_;
    moveit_msgs::PlanningScene original_scene_;

    /* store the planning scene diff temporarily*/
    moveit_msgs::PlanningScene planning_scene_diff_;
    ros::ServiceClient planning_scene_diff_client_;

    // random number generator
    std::mt19937 rng_;

};

#endif // APEX_MR_INSTANCE_H
